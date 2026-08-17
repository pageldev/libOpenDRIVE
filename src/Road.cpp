#include "libodr/Road.h"
#include "libodr/Sampler.h"

#include "libodr/Lane.h"
#include "libodr/Mesh.h"
#include "libodr/RefLine.h"
#include "libodr/RoadMark.h"
#include "libodr/RoadSignal.h"
#include "libodr/Utils.hpp"

#include "libodr/earcut.hpp"
#include "magic_enum/magic_enum.hpp"
#include "pugixml.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fmt/format.h>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace odr
{

double Crossfall::get(double s, bool on_left_side) const
{
    if (this->records.empty())
        return 0;

    auto target_record_iter = this->records.upper_bound(s);
    if (target_record_iter != this->records.begin())
        target_record_iter--;

    const Record& record = target_record_iter->second;

    if (on_left_side && record.side == Side::Right)
        return 0;
    else if (!on_left_side && record.side == Side::Left)
        return 0;

    return record.poly.evaluate(s);
}

RoadLink::RoadLink(const std::string& id, const std::string& type_str, std::optional<ContactPoint> contact_point) :
    id(id), contact_point(contact_point)
{
    std::optional<Type> type = magic_enum::enum_cast<Type>(type_str, magic_enum::case_insensitive);
    require_or_throw(type.has_value(), "road link type '{}' is invalid", type_str);
    if (*type == Type::Road)
        require_or_throw(contact_point.has_value(), "a road link of type 'road' requires a contact point");
    this->type = *type;
}

Speed::Speed(const std::string& max, const std::string& unit) : max(max), unit(unit) {}

Road::Road(
    const std::string& id, double length, const std::string& junction, std::optional<TrafficRule> traffic_rule, std::optional<std::string> name) :
    id(id), length(length), junction(junction), traffic_rule(traffic_rule), name(name), ref_line(length)
{
    require_or_throw(length > 0, "length must be greater than 0 (got {})", length);
}

double Road::get_lane_section_s(double s) const
{
    require_or_throw(!(this->s_to_lane_section.empty()), "road has no lane sections");

    auto lane_section_iter = this->s_to_lane_section.upper_bound(s); // first element > s
    if (lane_section_iter != this->s_to_lane_section.begin())
        lane_section_iter--;
    const double lane_section_s = lane_section_iter->first;
    require_or_throw(s >= lane_section_s, "s must not be before lane section start {} (got {})", lane_section_s, s);

    return lane_section_s;
}

double Road::get_lane_section_end(const LaneSection& lane_section) const
{
    auto lane_section_iter = this->s_to_lane_section.find(lane_section.s);
    require_or_throw(lane_section_iter != this->s_to_lane_section.end(), "no lane section found for s {}", lane_section.s);

    const bool is_last = (lane_section_iter == std::prev(this->s_to_lane_section.end()));
    if (is_last)
        return this->length;

    const double s_next = std::next(lane_section_iter)->first;
    return std::nextafter(s_next, -std::numeric_limits<double>::infinity()); // to be within lane section
}

double Road::get_lane_section_length(const LaneSection& lane_section) const
{
    const double s_end = this->get_lane_section_end(lane_section);
    return s_end - lane_section.s;
}

Vec3D Road::get_xyz(double s, double t, double h, Vec3D* _e_s, Vec3D* _e_t, Vec3D* _e_h, bool allow_extrapolate) const
{
    require_or_throw(allow_extrapolate || (s >= 0 && s <= this->length), "s must be in road range [0, {}] (got {})", this->length, s);
    const double s_clamped = std::min(std::max(s, 0.0), this->length);
    const Vec3D  s_vec = this->ref_line.derivative(s_clamped);
    const Vec3D  e_s = normalize(s_vec);

    const Vec3D e_t_base{-e_s[1], e_s[0], 0.0}; // flat in xy-plane and perpendicular to e_s
    const Vec3D e_h_base = crossProduct(e_s, e_t_base);

    // Rodrigues rotation of e_t_base around e_s by theta; simplified since dot(k,v)=0 and cross(k,v)=e_h_base
    const double theta = this->superelevation.evaluate(s_clamped).value_or(0.0);
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    const Vec3D  e_t = normalize(Vec3D{cos_theta * e_t_base[0] + sin_theta * e_h_base[0],
                                      cos_theta * e_t_base[1] + sin_theta * e_h_base[1],
                                      cos_theta * e_t_base[2] + sin_theta * e_h_base[2]});

    const Vec3D e_h = normalize(crossProduct(e_s, e_t));
    Vec3D       p0 = this->ref_line.get_xyz(s_clamped);
    if (s != s_clamped) // out of road bounds, linear extrapolate
        p0 = add(p0, mut(s - s_clamped, s_vec));

    const Vec3D xyz{p0[0] + t * e_t[0] + h * e_h[0], p0[1] + t * e_t[1] + h * e_h[1], p0[2] + t * e_t[2] + h * e_h[2]};

    if (_e_s)
        *_e_s = e_s;
    if (_e_t)
        *_e_t = e_t;
    if (_e_h)
        *_e_h = e_h;

    return xyz;
}

Vec3D Road::get_lane_surface_pt(double lane_section_s, double lane_id, double s, double t, Vec3D* vn, bool allow_extrapolate) const
{
    require_or_throw(allow_extrapolate || (s >= 0 && s <= this->length), "s must be in road range [0, {}] (got {})", this->length, s);
    const double s_clamped = std::min(std::max(s, 0.0), this->length);

    const LaneSection& lane_section = this->s_to_lane_section.at(lane_section_s);
    const Lane&        lane = lane_section.id_to_lane.at(lane_id);
    const Lane&        inner_neighbor_lane = lane_section.id_to_lane.at(next_towards_zero(lane.id));

    const std::optional<double> t_inner_brdr_opt = inner_neighbor_lane.outer_border.evaluate(s_clamped);
    require_or_throw(
        t_inner_brdr_opt.has_value() || inner_neighbor_lane.id == 0, "lane {} has no outer border at s {}", inner_neighbor_lane.id, s_clamped);
    const double t_inner_brdr = t_inner_brdr_opt.value_or(0.0);
    double       h = 0;

    // OpenDRIVE® Format Specification, Rev. 1.4, 5.3.7.2.1.1 Lane Record:
    // "keep lane on level, .i.e. do not apply superelevation or crossfall"
    if (lane.level.value_or(false))
    {
        // compensate crossfall and superelevation to level lane
        const double alpha = this->crossfall.get(s_clamped, (lane.id > 0));
        const double theta = this->superelevation.evaluate(s_clamped).value_or(0.0);
        h = -std::tan(alpha) * std::abs(t_inner_brdr) + std::tan(theta) * (t - t_inner_brdr);
    }
    else
    {
        h = -std::tan(this->crossfall.get(s_clamped, (lane.id > 0))) * std::abs(t);
    }

    // OpenDRIVE® Format Specification, Rev. 1.4, 5.3.7.2.1.1.9 Lane Height Record:
    // "The surface of a lane may be offset from the plane defined by the reference line and the corresponding elevation and crossfall entries"
    if (!lane.s_to_height_offset.empty())
    {
        const std::map<double, HeightOffset>& heights = lane.s_to_height_offset;

        const auto heights_iter = heights.upper_bound(s_clamped); // first element > s
        if (heights_iter != heights.begin())                      // s after first <height> record
        {
            const HeightOffset&         height_offset = std::prev(heights_iter)->second;
            const double                h_inner = height_offset.inner;
            const double                h_outer = height_offset.outer;
            const std::optional<double> t_outer_brdr = lane.outer_border.evaluate(s_clamped);
            require_or_throw(t_outer_brdr.has_value() || lane.id == 0, "lane {} has no outer border at s {}", lane.id, s_clamped);
            const double t_outer_brdr_value = t_outer_brdr.value_or(0.0);
            const double t_norm = (t_outer_brdr_value != t_inner_brdr) ? (t - t_inner_brdr) / (t_outer_brdr_value - t_inner_brdr) : 0.0; // [0,1]
            h += t_norm * (h_outer - h_inner) + h_inner;
        }
    }

    return this->get_xyz(s, t, h, nullptr, nullptr, vn, allow_extrapolate);
}

Vec3D Road::get_surface_pt(double s, double t, Vec3D* vn, bool allow_extrapolate) const
{
    const double       s_clamped = std::min(std::max(s, 0.0), this->length);
    const double       lane_section_s = this->get_lane_section_s(s_clamped);
    const LaneSection& lane_section = this->s_to_lane_section.at(lane_section_s);
    const double       lane_id = lane_section.get_lane_id(s_clamped, t);

    return this->get_lane_surface_pt(lane_section_s, lane_id, s, t, vn, allow_extrapolate);
}

Mesh3D Road::get_lane_mesh(double lane_section_s, int lane_id, double s_start, double s_end, double eps, std::vector<uint32_t>* outline_indices) const
{
    const LaneSection& lane_section = this->s_to_lane_section.at(lane_section_s);
    const Lane&        lane = lane_section.id_to_lane.at(lane_id);
    const Lane&        inner_lane = lane_section.id_to_lane.at(next_towards_zero(lane_id));

    const LaneSampler      lane_sampler(*this, lane_section_s, lane, inner_lane);
    const std::set<double> samples = lane_sampler.get_mesh_s_samples(s_start, s_end, eps);

    Mesh3D mesh;
    for (const double s : samples)
    {
        for (const Lane* edge_lane : {&lane, &inner_lane})
        {
            const std::optional<double> t = edge_lane->outer_border.evaluate(s);
            require_or_throw(t.has_value() || edge_lane->id == 0, "lane {} has no outer border at s {}", edge_lane->id, s);
            Vec3D normal{0, 0, 0};
            mesh.vertices.push_back(this->get_lane_surface_pt(lane_section_s, lane_id, s, t.value_or(0.0), &normal));
            mesh.normals.push_back(normal);
            mesh.st_coordinates.push_back({s, t.value_or(0.0)});
        }
    }

    const bool ccw = lane.id < 0;
    for (std::size_t idx = 3; idx < mesh.vertices.size(); idx += 2)
    {
        const std::array<std::size_t, 6> patch = ccw ? std::array<std::size_t, 6>{idx - 3, idx - 1, idx, idx - 3, idx, idx - 2}
                                                     : std::array<std::size_t, 6>{idx - 3, idx, idx - 1, idx - 3, idx - 2, idx};
        mesh.indices.insert(mesh.indices.end(), patch.begin(), patch.end());
    }
    if (outline_indices)
        *outline_indices = get_triangle_strip_outline_indices<uint32_t>(mesh.vertices.size());
    return mesh;
}

Mesh3D Road::get_lane_mesh(double lane_section_s, int lane_id, double eps, std::vector<uint32_t>* outline_indices) const
{
    const LaneSection& lane_section = this->s_to_lane_section.at(lane_section_s);
    return this->get_lane_mesh(lane_section_s, lane_id, lane_section.s, this->get_lane_section_end(lane_section), eps, outline_indices);
}

Mesh3D Road::get_roadmark_mesh(double lane_section_s, int lane_id, const SingleRoadMark& roadmark, double eps, bool enforce_road_bounds) const
{
    if (is_zero(roadmark.width))
        return Mesh3D{};

    const LaneSection& lane_section = this->s_to_lane_section.at(lane_section_s);
    const Lane&        lane = lane_section.id_to_lane.at(lane_id);
    const Lane&        inner_lane = lane_section.id_to_lane.at(next_towards_zero(lane.id));

    const std::set<double> s_samples =
        LaneSampler(*this, lane_section_s, lane, inner_lane).get_border_s_samples(roadmark.s_start, roadmark.s_end, roadmark.t_offset, eps);

    Mesh3D out_mesh;
    for (const double s : s_samples)
    {
        Vec3D                       vn_edge_a{0, 0, 0};
        const std::optional<double> t_lane_outer_border = lane.outer_border.evaluate(s);
        require_or_throw(t_lane_outer_border.has_value() || lane.id == 0, "lane {} has no outer border at s {}", lane.id, s);
        const double t_edge_a = t_lane_outer_border.value_or(0.0) + roadmark.width * 0.5 + roadmark.t_offset;
        out_mesh.vertices.push_back(this->get_lane_surface_pt(lane_section_s, lane_id, s, t_edge_a, &vn_edge_a, !enforce_road_bounds));
        out_mesh.normals.push_back(vn_edge_a);

        Vec3D        vn_edge_b{0, 0, 0};
        const double t_edge_b = t_edge_a - roadmark.width;
        out_mesh.vertices.push_back(this->get_lane_surface_pt(lane_section_s, lane_id, s, t_edge_b, &vn_edge_b, !enforce_road_bounds));
        out_mesh.normals.push_back(vn_edge_b);
    }

    const std::size_t num_pts = out_mesh.vertices.size();
    for (std::size_t idx = 3; idx < num_pts; idx += 2)
    {
        std::array<size_t, 6> indicies_patch = {idx - 3, idx, idx - 1, idx - 3, idx - 2, idx};
        out_mesh.indices.insert(out_mesh.indices.end(), indicies_patch.begin(), indicies_patch.end());
    }

    return out_mesh;
}

Mesh3D Road::get_road_signal_mesh(const RoadSignal& road_signal, bool enforce_road_bounds) const
{
    const Mat3D  rot_mat = EulerAnglesToMatrix<double>(road_signal.roll.value_or(0), road_signal.pitch.value_or(0), road_signal.hOffset.value_or(0));
    const double s = road_signal.s;
    const double t = road_signal.t;
    const double z_offset = road_signal.z_offset;
    const double height = road_signal.height.value_or(RoadSignal::DefaultHeight);
    const double width = road_signal.width.value_or(RoadSignal::DefaultWidth);

    Mesh3D road_signal_mesh = RoadSignal::get_box(width, RoadSignal::Thickness, height);

    Vec3D       e_s, e_t, e_h;
    const Vec3D p0 = this->get_xyz(s, t, z_offset, &e_s, &e_t, &e_h, !enforce_road_bounds);
    const Mat3D base_mat{{{e_s[0], e_t[0], e_h[0]}, {e_s[1], e_t[1], e_h[1]}, {e_s[2], e_t[2], e_h[2]}}};
    for (Vec3D& pt_uvz : road_signal_mesh.vertices)
    {
        pt_uvz = MatVecMultiplication(rot_mat, pt_uvz);
        pt_uvz = MatVecMultiplication(base_mat, pt_uvz);
        pt_uvz = add(pt_uvz, p0);
        road_signal_mesh.st_coordinates.push_back({s, t});
    }

    return road_signal_mesh;
}

Mesh3D Road::get_road_object_mesh(const RoadObject&         road_obj,
                                  double                    eps,
                                  double                    default_h,
                                  double                    default_z_offset,
                                  bool                      enforce_road_bounds,
                                  std::vector<std::string>* warnings) const
{
    std::vector<RoadObjectRepeat> repeats_copy = road_obj.repeats; // make copy to keep method const
    if (repeats_copy.empty() && road_obj.outlines.empty())         // single road object - no repeats or outlines, handle as one repeat
    {
        require_or_throw(road_obj.s.has_value(), "s-coordinate is required for a road object without repeats or outlines");
        require_or_throw(road_obj.t.has_value(), "t-coordinate is required for a road object without repeats or outlines");
        RoadObjectRepeat rp(*(road_obj.s),
                            0,
                            1,
                            *(road_obj.t),
                            *(road_obj.t),
                            road_obj.height.value_or(default_h),
                            road_obj.height.value_or(default_h),
                            road_obj.z_offset.value_or(default_z_offset),
                            road_obj.z_offset.value_or(default_z_offset),
                            road_obj.width,
                            road_obj.width);
        repeats_copy.push_back(rp);
    }

    const Mat3D rot_mat = EulerAnglesToMatrix<double>(road_obj.roll.value_or(0), road_obj.pitch.value_or(0), road_obj.hdg.value_or(0));

    Mesh3D road_obj_mesh;

    // outline objects are not repeated, repeats only apply to the generic road object (box or cylinder)
    // note: if road object has an outline object AND repeat this will create generic objects at repeat AND the outline object (non-repeated)
    for (const RoadObjectRepeat& r : repeats_copy)
    {
        const bool has_t_range = r.t_start.has_value() && r.t_end.has_value();
        require_or_throw(has_t_range || road_obj.t.has_value(), "t-coordinate is required for a road object repeat");

        const double s_start = r.s;
        const double s_end = std::min(s_start + r.length, this->length);
        const double t_start = has_t_range ? *r.t_start : *(road_obj.t);
        const double t_end = has_t_range ? *r.t_end : *(road_obj.t);
        const double height_start = r.height_start.value_or(road_obj.height.value_or(default_h));
        const double height_end = r.height_end.value_or(road_obj.height.value_or(default_h));
        const double z_offset_start = r.z_offset_start.value_or(road_obj.z_offset.value_or(default_z_offset));
        const double z_offset_end = r.z_offset_end.value_or(road_obj.z_offset.value_or(default_z_offset));
        const double width_start = r.width_start.value_or(road_obj.width.value_or(0));
        const double width_end = r.width_end.value_or(road_obj.width.value_or(0));

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
                if (road_obj.radius) // cylinder
                    single_road_obj_mesh = RoadObject::get_cylinder(eps, *(road_obj.radius), h_s);
                else if (road_obj.length && w_s > 0) // box
                    single_road_obj_mesh = RoadObject::get_cube(w_s, *(road_obj.length), h_s);
                else // fallback to cube
                {
                    if (warnings)
                        warnings->push_back("object has neither a radius nor both length and width: using a default cube");
                    single_road_obj_mesh = RoadObject::get_cube(0.1, 0.1, 0.1);
                }

                Vec3D       e_s, e_t, e_h;
                const Vec3D p0 = this->get_xyz(s, t_s, z_offset_s, &e_s, &e_t, &e_h, !enforce_road_bounds);
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

            const std::set<double> s_samples = RoadSampler(*this).get_polyline_s_samples(s_start, t_start, s_end, t_end, eps);
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
                    this->get_xyz(s, t_s - 0.5 * w_s, z_bottom, nullptr, nullptr, nullptr, !enforce_road_bounds));
                continuous_road_obj_mesh.vertices.push_back(
                    this->get_xyz(s, t_s + 0.5 * w_s, z_bottom, nullptr, nullptr, nullptr, !enforce_road_bounds));
                continuous_road_obj_mesh.vertices.push_back(
                    this->get_xyz(s, t_s + 0.5 * w_s, z_top, nullptr, nullptr, nullptr, !enforce_road_bounds));
                continuous_road_obj_mesh.vertices.push_back(
                    this->get_xyz(s, t_s - 0.5 * w_s, z_top, nullptr, nullptr, nullptr, !enforce_road_bounds));

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

    for (const RoadObjectOutline& road_object_outline : road_obj.outlines)
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
                    require_or_throw(road_obj.s.has_value(), "s-coordinate is required for a <cornerLocal> outline");
                    require_or_throw(road_obj.t.has_value(), "t-coordinate is required for a <cornerLocal> outline");
                    st_obj = {*(road_obj.s), *(road_obj.t)};
                    Vec3D       e_s, e_t, e_h;
                    const Vec3D p0 =
                        this->get_xyz(st_obj[0], st_obj[1], road_obj.z_offset.value_or(default_z_offset), &e_s, &e_t, &e_h, !enforce_road_bounds);
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
                    pt_obj = this->get_xyz(st_obj[0], st_obj[1], corner.pt[2] + h_obj, nullptr, nullptr, nullptr, !enforce_road_bounds);
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
