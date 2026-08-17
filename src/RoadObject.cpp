#include "libodr/RoadObject.h"
#include "libodr/Road.h"
#include "libodr/Sampler.h"
#include "libodr/Utils.hpp"

#include "fmt/format.h"
#include "libodr/earcut.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>

namespace odr
{

RoadObjectRepeat::RoadObjectRepeat(double                s,
                                   double                length,
                                   double                distance,
                                   std::optional<double> t_start,
                                   std::optional<double> t_end,
                                   std::optional<double> height_start,
                                   std::optional<double> height_end,
                                   std::optional<double> z_offset_start,
                                   std::optional<double> z_offset_end,
                                   std::optional<double> width_start,
                                   std::optional<double> width_end) :
    s(s),
    length(length),
    distance(distance),
    t_start(t_start),
    t_end(t_end),
    height_start(height_start),
    height_end(height_end),
    z_offset_start(z_offset_start),
    z_offset_end(z_offset_end),
    width_start(width_start),
    width_end(width_end)
{
    require_or_throw(s >= 0, "s must be greater than or equal to 0 (got {})", s);
    require_or_throw(length >= 0, "length must be greater than or equal to 0 (got {})", length);
    require_or_throw(distance >= 0, "distance must be greater than or equal to 0 (got {})", distance);
    require_or_throw(!t_start || !std::isnan(*t_start), "tStart must not be NaN");
    require_or_throw(!t_end || !std::isnan(*t_end), "tEnd must not be NaN");
    require_or_throw(!height_start || !std::isnan(*height_start), "heightStart must not be NaN"); // OpenDRIVE 1.4-1.6 allows negative height
    require_or_throw(!height_end || !std::isnan(*height_end), "heightEnd must not be NaN");
    require_or_throw(!width_start || width_start >= 0, "widthStart must be greater than or equal to 0");
    require_or_throw(!width_end || width_end >= 0, "widthEnd must be greater than or equal to 0");
}

RoadObjectCorner::RoadObjectCorner(Vec3D pt, double height, Type type, std::optional<int> id) : pt(pt), height(height), type(type), id(id)
{
    require_or_throw(std::none_of(pt.begin(), pt.end(), [](double v) { return std::isnan(v); }),
                     "point coordinates must not contain NaN values (got [{}])",
                     fmt::join(pt, ", "));
    require_or_throw(!std::isnan(height), "height must not be NaN"); // OpenDRIVE 1.4-1.6 allows negative height
    if (type == Type::Road)
        require_or_throw(pt[0] >= 0, "s must be greater than or equal to 0 (got {})", pt[0]);
}

RoadObjectOutline::RoadObjectOutline(std::optional<int>         id,
                                     std::optional<std::string> fill_type,
                                     std::optional<std::string> lane_type,
                                     std::optional<bool>        outer,
                                     std::optional<bool>        closed) :
    id(id), fill_type(fill_type), lane_type(lane_type), outer(outer), closed(closed)
{
}

RoadObject::RoadObject(const std::string&         id,
                       std::optional<double>      s,
                       std::optional<double>      t,
                       std::optional<double>      z_offset,
                       std::optional<double>      length,
                       std::optional<double>      valid_length,
                       std::optional<double>      width,
                       std::optional<double>      radius,
                       std::optional<double>      height,
                       std::optional<double>      hdg,
                       std::optional<double>      pitch,
                       std::optional<double>      roll,
                       std::optional<std::string> type,
                       std::optional<std::string> name,
                       std::optional<std::string> subtype,
                       std::optional<Orientation> orientation,
                       std::optional<bool>        is_dynamic) :

    id(id),
    s(s),
    t(t),
    z_offset(z_offset),
    length(length),
    valid_length(valid_length),
    width(width),
    radius(radius),
    height(height),
    hdg(hdg),
    pitch(pitch),
    roll(roll),
    type(type),
    name(name),
    subtype(subtype),
    orientation(orientation),
    is_dynamic(is_dynamic)
{
    require_or_throw(!s || s >= 0, "s must be greater than or equal to 0");
    require_or_throw(!t || !std::isnan(*t), "t must not be NaN");
    require_or_throw(!z_offset || !std::isnan(*z_offset), "zOffset must not be NaN");
    require_or_throw(!length || length > 0, "length must be greater than 0");
    require_or_throw(!valid_length || valid_length >= 0, "valid length must be greater than or equal to 0");
    require_or_throw(!width || !std::isnan(*width), "width must not be NaN");
    require_or_throw(!radius || radius > 0, "radius must be greater than 0");
    require_or_throw(!height || !std::isnan(*height), "height must not be NaN"); // OpenDRIVE 1.4-1.6 allows negative height
    require_or_throw(!hdg || !std::isnan(*hdg), "heading must not be NaN");
    require_or_throw(!pitch || !std::isnan(*pitch), "pitch must not be NaN");
    require_or_throw(!roll || !std::isnan(*roll), "roll must not be NaN");
}

Mesh3D RoadObject::get_cylinder(double eps, double radius, double height)
{
    require_or_throw(std::isfinite(eps) && eps > 0, "eps must be finite and greater than 0 (got {})", eps);

    Mesh3D       cylinder_mesh;
    const double z_bottom = std::min(0.0, height);
    const double z_top = std::max(0.0, height);
    cylinder_mesh.vertices.push_back({0, 0, z_bottom});
    cylinder_mesh.vertices.push_back({0, 0, z_top});

    const double eps_adj = 0.5 * eps; // reduce eps a bit, cylinders more subsceptible to low resolution
    const double eps_angle =
        (radius <= eps_adj) ? M_PI / 6 : std::acos((radius * radius - 4 * radius * eps_adj + 2 * eps_adj * eps_adj) / (radius * radius));

    std::vector<double> angles;
    for (double alpha = 0; alpha < 2 * M_PI; alpha += eps_angle)
        angles.push_back(alpha);
    angles.push_back(2 * M_PI);

    for (const double alpha : angles)
    {
        const Vec3D circle_pt_bottom = {radius * std::cos(alpha), radius * std::sin(alpha), z_bottom};
        const Vec3D circle_pt_top = {radius * std::cos(alpha), radius * std::sin(alpha), z_top};
        cylinder_mesh.vertices.push_back(circle_pt_bottom);
        cylinder_mesh.vertices.push_back(circle_pt_top);

        if (cylinder_mesh.vertices.size() > 5)
        {
            const std::size_t     cur_idx = cylinder_mesh.vertices.size() - 1;
            std::array<size_t, 6> top_bottom_idx_patch = {0, cur_idx - 1, cur_idx - 3, 1, cur_idx - 2, cur_idx};
            cylinder_mesh.indices.insert(cylinder_mesh.indices.end(), top_bottom_idx_patch.begin(), top_bottom_idx_patch.end());
            std::array<size_t, 6> wall_idx_patch = {cur_idx, cur_idx - 2, cur_idx - 3, cur_idx, cur_idx - 3, cur_idx - 1};
            cylinder_mesh.indices.insert(cylinder_mesh.indices.end(), wall_idx_patch.begin(), wall_idx_patch.end());
        }
    }

    return cylinder_mesh;
}

Mesh3D RoadObject::get_cube(double w, double l, double h)
{
    const double z_bottom = std::min(0.0, h);
    const double z_top = std::max(0.0, h);
    return Mesh3D({Vec3D{l / 2, w / 2, z_bottom},
                   Vec3D{-l / 2, w / 2, z_bottom},
                   Vec3D{-l / 2, -w / 2, z_bottom},
                   Vec3D{l / 2, -w / 2, z_bottom},
                   Vec3D{l / 2, w / 2, z_top},
                   Vec3D{-l / 2, w / 2, z_top},
                   Vec3D{-l / 2, -w / 2, z_top},
                   Vec3D{l / 2, -w / 2, z_top}},
                  {0, 3, 1, 3, 2, 1, 4, 5, 7, 7, 5, 6, 7, 6, 3, 3, 6, 2, 5, 4, 1, 1, 4, 0, 0, 4, 7, 7, 3, 0, 1, 6, 5, 1, 2, 6},
                  {},
                  {});
}

Mesh3D RoadObject::get_mesh(double eps, double default_h, double default_z_offset, bool enforce_road_bounds, std::vector<std::string>* warnings) const
{
    const Road& road = *get_parent_or_throw<Road>(*this);

    std::vector<RoadObjectRepeat> repeats_copy = this->repeats; // make copy to keep method const
    if (repeats_copy.empty() && this->outlines.empty())         // single road object - no repeats or outlines, handle as one repeat
    {
        require_or_throw(this->s.has_value(), "s-coordinate is required for a road object without repeats or outlines");
        require_or_throw(this->t.has_value(), "t-coordinate is required for a road object without repeats or outlines");
        RoadObjectRepeat rp(*(this->s),
                            0,
                            1,
                            *(this->t),
                            *(this->t),
                            this->height.value_or(default_h),
                            this->height.value_or(default_h),
                            this->z_offset.value_or(default_z_offset),
                            this->z_offset.value_or(default_z_offset),
                            this->width,
                            this->width);
        repeats_copy.push_back(rp);
    }

    const Mat3D rot_mat = EulerAnglesToMatrix<double>(this->roll.value_or(0), this->pitch.value_or(0), this->hdg.value_or(0));

    Mesh3D road_obj_mesh;

    // outline objects are not repeated, repeats only apply to the generic road object (box or cylinder)
    // note: if road object has an outline object AND repeat this will create generic objects at repeat AND the outline object (non-repeated)
    for (const RoadObjectRepeat& r : repeats_copy)
    {
        const bool has_t_range = r.t_start.has_value() && r.t_end.has_value();
        require_or_throw(has_t_range || this->t.has_value(), "t-coordinate is required for a road object repeat");

        const double s_start = r.s;
        const double s_end = std::min(s_start + r.length, road.length);
        const double t_start = has_t_range ? *r.t_start : *(this->t);
        const double t_end = has_t_range ? *r.t_end : *(this->t);
        const double height_start = r.height_start.value_or(this->height.value_or(default_h));
        const double height_end = r.height_end.value_or(this->height.value_or(default_h));
        const double z_offset_start = r.z_offset_start.value_or(this->z_offset.value_or(default_z_offset));
        const double z_offset_end = r.z_offset_end.value_or(this->z_offset.value_or(default_z_offset));
        const double width_start = r.width_start.value_or(this->width.value_or(0));
        const double width_end = r.width_end.value_or(this->width.value_or(0));

        // OpenDRIVE Format Specification, Rev. 1.4, 5.3.8.1.1 Object Repeat Record:
        // "distance between two instances of the object;
        // If this value is zero, then the object is considered to be a continuous feature like a guard rail, a wall etc."
        if (!is_zero(r.distance)) // non-continuous object
        {
            for (double s = s_start; s <= s_end; s += r.distance)
            {
                const double p = (s_end == s_start) ? 1.0 : (s - s_start) / (s_end - s_start);
                const double t_s = t_start + p * (t_end - t_start);
                const double h_s = height_start + p * (height_end - height_start);
                const double z_offset_s = z_offset_start + p * (z_offset_end - z_offset_start);
                const double w_s = width_start + p * (width_end - width_start);

                Mesh3D single_road_obj_mesh;
                if (this->radius) // cylinder
                    single_road_obj_mesh = RoadObject::get_cylinder(eps, *(this->radius), h_s);
                else if (this->length && w_s > 0) // box
                    single_road_obj_mesh = RoadObject::get_cube(w_s, *(this->length), h_s);
                else // fallback to cube
                {
                    if (warnings)
                        warnings->push_back("object has neither a radius nor both length and width: using a default cube");
                    single_road_obj_mesh = RoadObject::get_cube(0.1, 0.1, 0.1);
                }

                Vec3D       e_s, e_t, e_h;
                const Vec3D p0 = road.get_xyz(s, t_s, z_offset_s, &e_s, &e_t, &e_h, !enforce_road_bounds);
                const Mat3D base_mat{{{e_s[0], e_t[0], e_h[0]}, {e_s[1], e_t[1], e_h[1]}, {e_s[2], e_t[2], e_h[2]}}};
                for (Vec3D& pt_uvz : single_road_obj_mesh.vertices)
                {
                    pt_uvz = MatVecMultiplication(rot_mat, pt_uvz);
                    pt_uvz = MatVecMultiplication(base_mat, pt_uvz);
                    pt_uvz = add(pt_uvz, p0);
                    single_road_obj_mesh.st_coordinates.push_back({s, t_s});
                }

                road_obj_mesh.add_mesh(single_road_obj_mesh);
            }
        }
        else // continuous object
        {
            Mesh3D continuous_road_obj_mesh;

            const std::array<size_t, 24> idx_patch_template = {1, 5, 4, 1, 4, 0, 2, 7, 6, 2, 3, 7, 1, 6, 5, 1, 2, 6, 0, 4, 7, 0, 7, 3};

            const std::set<double> s_samples = RoadSampler(road).get_polyline_s_samples(s_start, t_start, s_end, t_end, eps);
            for (const double s : s_samples)
            {
                const double p = (s_end == s_start) ? 1.0 : (s - s_start) / (s_end - s_start);
                const double t_s = t_start + p * (t_end - t_start);
                const double h_s = height_start + p * (height_end - height_start);
                const double z_offset_s = z_offset_start + p * (z_offset_end - z_offset_start);
                const double w_s = width_start + p * (width_end - width_start);
                const double z_bottom = z_offset_s + std::min(0.0, h_s);
                const double z_top = z_offset_s + std::max(0.0, h_s);

                continuous_road_obj_mesh.vertices.push_back(
                    road.get_xyz(s, t_s - 0.5 * w_s, z_bottom, nullptr, nullptr, nullptr, !enforce_road_bounds));
                continuous_road_obj_mesh.vertices.push_back(
                    road.get_xyz(s, t_s + 0.5 * w_s, z_bottom, nullptr, nullptr, nullptr, !enforce_road_bounds));
                continuous_road_obj_mesh.vertices.push_back(road.get_xyz(s, t_s + 0.5 * w_s, z_top, nullptr, nullptr, nullptr, !enforce_road_bounds));
                continuous_road_obj_mesh.vertices.push_back(road.get_xyz(s, t_s - 0.5 * w_s, z_top, nullptr, nullptr, nullptr, !enforce_road_bounds));

                const std::array<Vec2D, 4> s_t_coords = {{{s, t_s - 0.5 * w_s}, {s, t_s + 0.5 * w_s}, {s, t_s + 0.5 * w_s}, {s, t_s - 0.5 * w_s}}};
                continuous_road_obj_mesh.st_coordinates.insert(continuous_road_obj_mesh.st_coordinates.end(), s_t_coords.begin(), s_t_coords.end());

                if (continuous_road_obj_mesh.vertices.size() == 4)
                {
                    const std::array<size_t, 6> front_idx_patch = {0, 2, 1, 0, 3, 2};
                    continuous_road_obj_mesh.indices.insert(continuous_road_obj_mesh.indices.end(), front_idx_patch.begin(), front_idx_patch.end());
                }

                if (continuous_road_obj_mesh.vertices.size() > 7)
                {
                    const std::size_t      cur_offs = continuous_road_obj_mesh.vertices.size() - 8;
                    std::array<size_t, 24> wall_idx_patch;
                    for (std::size_t idx = 0; idx < idx_patch_template.size(); idx++)
                        wall_idx_patch.at(idx) = idx_patch_template.at(idx) + cur_offs;
                    continuous_road_obj_mesh.indices.insert(continuous_road_obj_mesh.indices.end(), wall_idx_patch.begin(), wall_idx_patch.end());
                }
            }
            if (continuous_road_obj_mesh.vertices.empty()) // can happen for e.g. s_start == s_end
                continue;

            const std::size_t           last_idx = continuous_road_obj_mesh.vertices.size() - 1;
            const std::array<size_t, 6> back_idx_patch = {last_idx - 3, last_idx - 2, last_idx - 1, last_idx - 3, last_idx - 1, last_idx};
            continuous_road_obj_mesh.indices.insert(continuous_road_obj_mesh.indices.end(), back_idx_patch.begin(), back_idx_patch.end());

            road_obj_mesh.add_mesh(continuous_road_obj_mesh);
        }
    }

    for (const RoadObjectOutline& road_object_outline : this->outlines)
    {
        // can't add point or line object
        if (road_object_outline.outline.size() < 3)
        {
            if (warnings)
                warnings->push_back("cannot create an outline with fewer than 3 points");
            continue;
        }

        Mesh3D outline_road_obj_mesh;

        // add top outline first - ensure the top vertices are at the front
        const bool is_flat_object = std::all_of(
            road_object_outline.outline.begin(), road_object_outline.outline.end(), [](const RoadObjectCorner& c) { return is_zero(c.height); });
        for (const bool is_top : {true, false})
        {
            if (is_flat_object && is_top)
                continue;
            for (const RoadObjectCorner& corner : road_object_outline.outline)
            {
                const double h_obj = is_top ? std::max(0.0, corner.height) : std::min(0.0, corner.height);

                Vec3D pt_obj;
                Vec2D st_obj;
                if (corner.type == RoadObjectCorner::Type::Local_AbsZ || corner.type == RoadObjectCorner::Type::Local_RelZ)
                {
                    require_or_throw(this->s.has_value(), "s-coordinate is required for a <cornerLocal> outline");
                    require_or_throw(this->t.has_value(), "t-coordinate is required for a <cornerLocal> outline");
                    st_obj = {*(this->s), *(this->t)};
                    Vec3D       e_s, e_t, e_h;
                    const Vec3D p0 =
                        road.get_xyz(st_obj[0], st_obj[1], this->z_offset.value_or(default_z_offset), &e_s, &e_t, &e_h, !enforce_road_bounds);
                    const Mat3D base_mat{{{e_s[0], e_t[0], e_h[0]}, {e_s[1], e_t[1], e_h[1]}, {e_s[2], e_t[2], e_h[2]}}};
                    pt_obj = {corner.pt[0], corner.pt[1], corner.pt[2]};
                    if (corner.type == RoadObjectCorner::Type::Local_AbsZ)
                        pt_obj[2] -= p0[2]; // make road relative
                    pt_obj = add(pt_obj, Vec3D{0, 0, h_obj});
                    pt_obj = add(MatVecMultiplication(base_mat, MatVecMultiplication(rot_mat, pt_obj)), p0);
                }
                else
                {
                    st_obj = {corner.pt[0], corner.pt[1]};
                    pt_obj = road.get_xyz(st_obj[0], st_obj[1], corner.pt[2] + h_obj, nullptr, nullptr, nullptr, !enforce_road_bounds);
                }

                outline_road_obj_mesh.vertices.push_back(pt_obj);
                outline_road_obj_mesh.st_coordinates.push_back(st_obj);
            }
        }

        // run 2D triangulation on top vertices
        const std::vector<size_t> idx_patch_top = mapbox::earcut<size_t>(outline_road_obj_mesh.vertices.data(), road_object_outline.outline.size());
        outline_road_obj_mesh.indices.insert(outline_road_obj_mesh.indices.end(), idx_patch_top.begin(), idx_patch_top.end());

        // add walls
        if (!is_flat_object)
        {
            const std::size_t N = road_object_outline.outline.size();
            for (std::size_t idx = 0; idx < N - 1; idx++)
            {
                std::array<size_t, 6> wall_idx_patch = {idx, idx + N, idx + 1, idx + 1, idx + N, idx + N + 1};
                outline_road_obj_mesh.indices.insert(outline_road_obj_mesh.indices.end(), wall_idx_patch.begin(), wall_idx_patch.end());
            }

            std::array<size_t, 6> last_idx_patch = {N - 1, 2 * N - 1, 0, 0, 2 * N - 1, N};
            outline_road_obj_mesh.indices.insert(outline_road_obj_mesh.indices.end(), last_idx_patch.begin(), last_idx_patch.end());
        }

        road_obj_mesh.add_mesh(outline_road_obj_mesh);
    }

    return road_obj_mesh;
}

} // namespace odr
