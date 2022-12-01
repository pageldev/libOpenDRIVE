#include "Road.h"
#include "Lane.h"
#include "RefLine.h"
#include "RoadMark.h"
#include "Utils.hpp"

#include "earcut/earcut.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace odr
{
double Crossfall::get_crossfall(const double s, const bool on_left_side) const
{
    if (this->s0_to_poly.size() > 0)
    {
        auto target_poly_iter = this->s0_to_poly.upper_bound(s);
        if (target_poly_iter != this->s0_to_poly.begin())
            target_poly_iter--;

        Side side = Side_Both; // applicable side of the road
        if (this->sides.find(target_poly_iter->first) != this->sides.end())
            side = this->sides.at(target_poly_iter->first);

        if (on_left_side && side == Side_Right)
            return 0;
        else if (!on_left_side && side == Side_Left)
            return 0;

        return target_poly_iter->second.get(s);
    }

    return 0;
}

RoadLink::RoadLink(std::string id, Type type, ContactPoint contact_point) : id(id), type(type), contact_point(contact_point) {}

RoadNeighbor::RoadNeighbor(std::string id, std::string side, std::string direction) : id(id), side(side), direction(direction) {}

SpeedRecord::SpeedRecord(std::string max, std::string unit) : max(max), unit(unit) {}

std::vector<LaneSection> Road::get_lanesections() const { return get_map_values(this->s_to_lanesection); }

std::vector<RoadObject> Road::get_road_objects() const { return get_map_values(this->id_to_object); }

Road::Road(std::string id, double length, std::string junction, std::string name) :
    length(length), id(id), junction(junction), name(name), ref_line(id, length)
{
}

double Road::get_lanesection_s0(const double s) const
{
    if (this->s_to_lanesection.empty())
        return NAN;

    auto s_lanesec_iter = this->s_to_lanesection.upper_bound(s);
    if (s_lanesec_iter != this->s_to_lanesection.begin())
        s_lanesec_iter--;
    const LaneSection& lanesec = s_lanesec_iter->second;

    if (s < lanesec.s0 || s > this->get_lanesection_end(lanesec))
        return NAN;

    return lanesec.s0;
}

LaneSection Road::get_lanesection(const double s) const
{
    const double lanesec_s0 = this->get_lanesection_s0(s);
    if (std::isnan(lanesec_s0))
        throw std::runtime_error("no valid lanesection");
    return this->s_to_lanesection.at(lanesec_s0);
}

double Road::get_lanesection_end(const LaneSection& lanesection) const { return this->get_lanesection_end(lanesection.s0); }

double Road::get_lanesection_end(const double lanesection_s0) const
{
    auto s_lanesec_iter = this->s_to_lanesection.find(lanesection_s0);
    if (s_lanesec_iter == this->s_to_lanesection.end())
        return NAN;

    const bool   is_last = (s_lanesec_iter == std::prev(this->s_to_lanesection.end()));
    const double next_s0 = is_last ? this->length : std::next(s_lanesec_iter)->first;

    return next_s0 - std::numeric_limits<double>::min(); // should be within lane section
}

double Road::get_lanesection_length(const LaneSection& lanesection) const
{
    const double s_end = this->get_lanesection_end(lanesection);
    return s_end - lanesection.s0;
}

double Road::get_lanesection_length(const double lanesection_s0) const
{
    const double s_end = this->get_lanesection_end(lanesection_s0);
    return s_end - lanesection_s0;
}

Vec3D Road::get_xyz(const double s, const double t, const double h, Vec3D* _e_s, Vec3D* _e_t, Vec3D* _e_h) const
{
    const Vec3D  s_vec = this->ref_line.get_grad(s);
    const double theta = this->superelevation.get(s);

    const Vec3D e_s = normalize(s_vec);
    const Vec3D e_t = normalize(Vec3D{std::cos(theta) * -e_s[1] + std::sin(theta) * -e_s[2] * e_s[0],
                                      std::cos(theta) * e_s[0] + std::sin(theta) * -e_s[2] * e_s[1],
                                      std::sin(theta) * (e_s[0] * e_s[0] + e_s[1] * e_s[1])});
    const Vec3D e_h = normalize(crossProduct(s_vec, e_t));
    const Vec3D p0 = this->ref_line.get_xyz(s);
    const Mat3D trans_mat{{{e_t[0], e_h[0], p0[0]}, {e_t[1], e_h[1], p0[1]}, {e_t[2], e_h[2], p0[2]}}};

    const Vec3D xyz = MatVecMultiplication(trans_mat, Vec3D{t, h, 1});

    if (_e_s)
        *_e_s = e_s;
    if (_e_t)
        *_e_t = e_t;
    if (_e_h)
        *_e_h = e_h;

    return xyz;
}

Vec3D Road::get_surface_pt(double s, const double t, Vec3D* vn) const
{
    CHECK_AND_REPAIR(s >= 0, "s < 0", s = 0);
    CHECK_AND_REPAIR(s <= this->length, "s > Road::length", s = this->length);

    const double lanesection_s0 = this->get_lanesection_s0(s);
    if (std::isnan(lanesection_s0))
    {
        throw std::runtime_error(string_format("cannot get road surface pt, no lane section for s %.3f, road length: %.3f", s, this->length));
    }

    const LaneSection& lanesection = this->s_to_lanesection.at(lanesection_s0);
    const Lane&        lane = lanesection.id_to_lane.at(lanesection.get_lane_id(s, t));
    const double       t_inner_brdr = lane.inner_border.get(s);
    double             h_t = 0;

    if (lane.level)
    {
        const double h_inner_brdr = -std::tan(this->crossfall.get_crossfall(s, (lane.id > 0))) * std::abs(t_inner_brdr);
        const double superelev = this->superelevation.get(s); // cancel out superelevation
        h_t = h_inner_brdr + std::tan(superelev) * (t - t_inner_brdr);
    }
    else
    {
        h_t = -std::tan(this->crossfall.get_crossfall(s, (lane.id > 0))) * std::abs(t);
    }

    if (lane.s_to_height_offset.size() > 0)
    {
        const std::map<double, HeightOffset>& height_offs = lane.s_to_height_offset;

        auto s0_height_offs_iter = height_offs.upper_bound(s);
        if (s0_height_offs_iter != height_offs.begin())
            s0_height_offs_iter--;

        const double t_outer_brdr = lane.outer_border.get(s);
        const double inner_height = s0_height_offs_iter->second.inner;
        const double outer_height = s0_height_offs_iter->second.outer;
        const double p_t = (t_outer_brdr != t_inner_brdr) ? (t - t_inner_brdr) / (t_outer_brdr - t_inner_brdr) : 0.0;
        h_t += p_t * (outer_height - inner_height) + inner_height;

        if (std::next(s0_height_offs_iter) != height_offs.end())
        {
            /* if successive lane height entry available linearly interpolate */
            const double ds = std::next(s0_height_offs_iter)->first - s0_height_offs_iter->first;
            const double d_lh_inner = std::next(s0_height_offs_iter)->second.inner - inner_height;
            const double dh_inner = (d_lh_inner / ds) * (s - s0_height_offs_iter->first);
            const double d_lh_outer = std::next(s0_height_offs_iter)->second.outer - outer_height;
            const double dh_outer = (d_lh_outer / ds) * (s - s0_height_offs_iter->first);

            h_t += p_t * (dh_outer - dh_inner) + dh_inner;
        }
    }

    return this->get_xyz(s, t, h_t, nullptr, nullptr, vn);
}

std::set<double>
Road::approximate_lane_border_linear(const Lane& lane, const double s_start, const double s_end, const double eps, const bool outer) const
{
    std::set<double> s_vals = this->ref_line.approximate_linear(eps, s_start, s_end);

    const CubicSpline& border = outer ? lane.outer_border : lane.inner_border;
    std::set<double>   s_vals_brdr = border.approximate_linear(eps, s_start, s_end);
    s_vals.insert(s_vals_brdr.begin(), s_vals_brdr.end());

    std::set<double> s_vals_lane_height = get_map_keys(lane.s_to_height_offset);
    s_vals.insert(s_vals_lane_height.begin(), s_vals_lane_height.end());

    const double     t_max = lane.outer_border.get_max(s_start, s_end);
    std::set<double> s_vals_superelev = this->superelevation.approximate_linear(std::atan(eps / std::abs(t_max)), s_start, s_end);
    s_vals.insert(s_vals_superelev.begin(), s_vals_superelev.end());

    return s_vals;
}

std::set<double> Road::approximate_lane_border_linear(const Lane& lane, const double eps, const bool outer) const
{
    const double s_end = this->get_lanesection_end(lane.key.lanesection_s0);
    return this->approximate_lane_border_linear(lane, lane.key.lanesection_s0, s_end, outer);
}

Line3D Road::get_lane_border_line(const Lane& lane, const double s_start, const double s_end, const double eps, const bool outer) const
{
    std::set<double> s_vals = this->approximate_lane_border_linear(lane, s_start, s_end, eps, outer);

    Line3D border_line;
    for (const double& s : s_vals)
    {
        const double t = outer ? lane.outer_border.get(s) : lane.inner_border.get(s);
        border_line.push_back(this->get_surface_pt(s, t));
    }

    return border_line;
}

Line3D Road::get_lane_border_line(const Lane& lane, const double eps, const bool outer) const
{
    const double s_end = this->get_lanesection_end(lane.key.lanesection_s0);
    return this->get_lane_border_line(lane, lane.key.lanesection_s0, s_end, eps, outer);
}

Mesh3D Road::get_lane_mesh(const Lane& lane, const double s_start, const double s_end, const double eps, std::vector<uint32_t>* outline_indices) const
{
    std::set<double> s_vals = this->ref_line.approximate_linear(eps, s_start, s_end);
    std::set<double> s_vals_outer_brdr = lane.outer_border.approximate_linear(eps, s_start, s_end);
    s_vals.insert(s_vals_outer_brdr.begin(), s_vals_outer_brdr.end());
    std::set<double> s_vals_inner_brdr = lane.inner_border.approximate_linear(eps, s_start, s_end);
    s_vals.insert(s_vals_inner_brdr.begin(), s_vals_inner_brdr.end());
    std::set<double> s_vals_lane_offset = this->lane_offset.approximate_linear(eps, s_start, s_end);
    s_vals.insert(s_vals_lane_offset.begin(), s_vals_lane_offset.end());

    std::set<double> s_vals_lane_height = get_map_keys(lane.s_to_height_offset);
    s_vals.insert(s_vals_lane_height.begin(), s_vals_lane_height.end());

    const double     t_max = lane.outer_border.get_max(s_start, s_end);
    std::set<double> s_vals_superelev = this->superelevation.approximate_linear(std::atan(eps / std::abs(t_max)), s_start, s_end);
    s_vals.insert(s_vals_superelev.begin(), s_vals_superelev.end());

    /* thin out s_vals array, be removing s vals closer than eps to each other */
    for (auto s_iter = s_vals.begin(); s_iter != s_vals.end();)
    {
        if (std::next(s_iter) != s_vals.end() && std::next(s_iter, 2) != s_vals.end() && ((*std::next(s_iter)) - *s_iter) <= eps)
            s_iter = std::prev(s_vals.erase(std::next(s_iter)));
        else
            s_iter++;
    }

    Mesh3D out_mesh;
    for (const double& s : s_vals)
    {
        Vec3D        vn_inner_brdr{0, 0, 0};
        const double t_inner_brdr = lane.inner_border.get(s);
        out_mesh.vertices.push_back(this->get_surface_pt(s, t_inner_brdr, &vn_inner_brdr));
        out_mesh.normals.push_back(vn_inner_brdr);
        out_mesh.st_coordinates.push_back({s, t_inner_brdr});

        Vec3D        vn_outer_brdr{0, 0, 0};
        const double t_outer_brdr = lane.outer_border.get(s);
        out_mesh.vertices.push_back(this->get_surface_pt(s, t_outer_brdr, &vn_outer_brdr));
        out_mesh.normals.push_back(vn_outer_brdr);
        out_mesh.st_coordinates.push_back({s, t_outer_brdr});
    }

    const std::size_t num_pts = out_mesh.vertices.size();
    const bool        ccw = lane.id > 0;
    for (std::size_t idx = 3; idx < num_pts; idx += 2)
    {
        std::array<size_t, 6> indicies_patch;
        if (ccw)
            indicies_patch = {idx - 3, idx - 1, idx, idx - 3, idx, idx - 2};
        else
            indicies_patch = {idx - 3, idx, idx - 1, idx - 3, idx - 2, idx};
        out_mesh.indices.insert(out_mesh.indices.end(), indicies_patch.begin(), indicies_patch.end());
    }

    if (outline_indices)
    {
        *outline_indices = get_triangle_strip_outline_indices<uint32_t>(out_mesh.vertices.size());
    }

    return out_mesh;
}

Mesh3D Road::get_lane_mesh(const Lane& lane, const double eps, std::vector<uint32_t>* outline_indices) const
{
    const double s_end = this->get_lanesection_end(lane.key.lanesection_s0);
    return this->get_lane_mesh(lane, lane.key.lanesection_s0, s_end, eps, outline_indices);
}

Mesh3D Road::get_roadmark_mesh(const Lane& lane, const RoadMark& roadmark, const double eps) const
{
    const std::set<double> s_vals = this->approximate_lane_border_linear(lane, roadmark.s_start, roadmark.s_end, eps, true);

    Mesh3D out_mesh;
    for (const double& s : s_vals)
    {
        Vec3D        vn_edge_a{0, 0, 0};
        const double t_edge_a = lane.outer_border.get(s) + roadmark.width * 0.5 + roadmark.t_offset;
        out_mesh.vertices.push_back(this->get_surface_pt(s, t_edge_a, &vn_edge_a));
        out_mesh.normals.push_back(vn_edge_a);

        Vec3D        vn_edge_b{0, 0, 0};
        const double t_edge_b = t_edge_a - roadmark.width;
        out_mesh.vertices.push_back(this->get_surface_pt(s, t_edge_b, &vn_edge_b));
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

Mesh3D Road::get_road_object_mesh(const RoadObject& road_object, const double eps) const
{
    std::vector<RoadObjectRepeat> repeats_copy = road_object.repeats; // make copy to keep method const
    if (repeats_copy.empty() && road_object.outline.empty())          // handle single object as 1 object repeat
    {
        RoadObjectRepeat rp(NAN, 0, 1, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN);
        rp.xml_node = this->xml_node;
        repeats_copy.push_back(rp);
    }

    const Mat3D rot_mat = EulerAnglesToMatrix<double>(road_object.roll, road_object.pitch, road_object.hdg);

    /* helper functions - object repeat's attributes override object's attributes */
    auto get_t_s = [&](const RoadObjectRepeat& r, const double& p) -> double
    { return (std::isnan(r.t_start) || std::isnan(r.t_end)) ? road_object.t0 : r.t_start + p * (r.t_end - r.t_start); };

    auto get_z_s = [&](const RoadObjectRepeat& r, const double& p) -> double
    {
        return (std::isnan(r.z_offset_start) || std::isnan(r.z_offset_end)) ? road_object.z0
                                                                            : r.z_offset_start + p * (r.z_offset_end - r.z_offset_start);
    };

    auto get_height_s = [&](const RoadObjectRepeat& r, const double& p) -> double
    { return (std::isnan(r.height_start) || std::isnan(r.height_end)) ? road_object.height : r.height_start + p * (r.height_end - r.height_start); };

    auto get_width_s = [&](const RoadObjectRepeat& r, const double& p) -> double
    { return (std::isnan(r.width_start) || std::isnan(r.width_end)) ? road_object.width : r.width_start + p * (r.width_end - r.width_start); };

    Mesh3D road_obj_mesh;

    for (const RoadObjectRepeat& repeat : repeats_copy)
    {
        const double s_start = std::isnan(repeat.s0) ? road_object.s0 : repeat.s0;
        const double s_end = std::min(s_start + repeat.length, this->length);

        if (repeat.distance != 0)
        {
            for (double s = s_start; s <= s_end; s += repeat.distance)
            {
                const double p = (s_end == s_start) ? 1.0 : (s - s_start) / (s_end - s_start);
                const double t_s = get_t_s(repeat, p);
                const double z_s = get_z_s(repeat, p);
                const double height_s = get_height_s(repeat, p);
                const double w_s = get_width_s(repeat, p);

                Mesh3D single_road_obj_mesh;
                if (road_object.radius > 0)
                {
                    single_road_obj_mesh = road_object.get_cylinder(eps, road_object.radius, height_s);
                }
                else if (w_s > 0 && road_object.length > 0)
                {
                    single_road_obj_mesh = road_object.get_box(w_s, road_object.length, height_s);
                }

                Vec3D       e_s, e_t, e_h;
                const Vec3D p0 = this->get_xyz(s, t_s, z_s, &e_s, &e_t, &e_h);
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
        else
        {
            Mesh3D continuous_road_obj_mesh;

            const std::array<size_t, 24> idx_patch_template = {1, 5, 4, 1, 4, 0, 2, 7, 6, 2, 3, 7, 1, 6, 5, 1, 2, 6, 0, 4, 7, 0, 7, 3};
            for (const double& s : this->ref_line.approximate_linear(eps, s_start, s_end))
            {
                const double p = (s_end == s_start) ? 1.0 : (s - s_start) / (s_end - s_start);
                const double t_s = get_t_s(repeat, p);
                const double z_s = get_z_s(repeat, p);
                const double height_s = get_height_s(repeat, p);
                const double w_s = get_width_s(repeat, p);

                continuous_road_obj_mesh.vertices.push_back(this->get_xyz(s, t_s - 0.5 * w_s, z_s));
                continuous_road_obj_mesh.vertices.push_back(this->get_xyz(s, t_s + 0.5 * w_s, z_s));
                continuous_road_obj_mesh.vertices.push_back(this->get_xyz(s, t_s + 0.5 * w_s, z_s + height_s));
                continuous_road_obj_mesh.vertices.push_back(this->get_xyz(s, t_s - 0.5 * w_s, z_s + height_s));

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

            const std::size_t           last_idx = continuous_road_obj_mesh.vertices.size() - 1;
            const std::array<size_t, 6> back_idx_patch = {last_idx - 3, last_idx - 2, last_idx - 1, last_idx - 3, last_idx - 1, last_idx};
            continuous_road_obj_mesh.indices.insert(continuous_road_obj_mesh.indices.end(), back_idx_patch.begin(), back_idx_patch.end());

            road_obj_mesh.add_mesh(continuous_road_obj_mesh);
        }
    }

    if (road_object.outline.size() > 1)
    {
        Vec3D       e_s, e_t, e_h;
        const Vec3D p0 = this->get_xyz(road_object.s0, road_object.t0, road_object.z0, &e_s, &e_t, &e_h);

        const Mat3D base_mat{{{e_s[0], e_t[0], e_h[0]}, {e_s[1], e_t[1], e_h[1]}, {e_s[2], e_t[2], e_h[2]}}};

        Mesh3D outline_road_obj_mesh;

        /* add top outline first - ensure the top vertices are at the front */
        const bool is_flat_object =
            std::all_of(road_object.outline.begin(), road_object.outline.end(), [](const RoadObjectCorner& c) { return c.height == 0; });
        if (!is_flat_object)
        {
            for (const RoadObjectCorner& corner : road_object.outline)
            {
                Vec3D pt_top;
                if (corner.type == RoadObjectCorner::Type_Local_AbsZ || corner.type == RoadObjectCorner::Type_Local_RelZ)
                {
                    pt_top = {corner.pt[0], corner.pt[1], corner.pt[2]};
                    if (corner.type == RoadObjectCorner::Type_Local_AbsZ)
                        pt_top[2] -= p0[2]; // make road relative
                    pt_top = add(pt_top, Vec3D{0, 0, corner.height});
                    pt_top = add(MatVecMultiplication(base_mat, MatVecMultiplication(rot_mat, pt_top)), p0);
                }
                else
                {
                    pt_top = this->get_xyz(corner.pt[0], corner.pt[1], corner.pt[2] + corner.height);
                }

                outline_road_obj_mesh.vertices.push_back(pt_top);
                outline_road_obj_mesh.st_coordinates.push_back({road_object.s0, road_object.t0});
            }
        }

        /* add bottom outline */
        for (const RoadObjectCorner& corner : road_object.outline)
        {
            Vec3D pt_base;
            if (corner.type == RoadObjectCorner::Type_Local_AbsZ || corner.type == RoadObjectCorner::Type_Local_RelZ)
            {
                pt_base = {corner.pt[0], corner.pt[1], corner.pt[2]};
                if (corner.type == RoadObjectCorner::Type_Local_AbsZ)
                    pt_base[2] -= p0[2]; // make road relative
                pt_base = add(MatVecMultiplication(base_mat, MatVecMultiplication(rot_mat, pt_base)), p0);
            }
            else
            {
                pt_base = this->get_xyz(corner.pt[0], corner.pt[1], corner.pt[2]);
            }

            outline_road_obj_mesh.vertices.push_back(pt_base);
            outline_road_obj_mesh.st_coordinates.push_back({road_object.s0, road_object.t0});
        }

        /* run 2D triangulation on top vertices */
        const std::vector<size_t> idx_patch_top = mapbox::earcut<size_t>(outline_road_obj_mesh.vertices.data(), road_object.outline.size());
        outline_road_obj_mesh.indices.insert(outline_road_obj_mesh.indices.end(), idx_patch_top.begin(), idx_patch_top.end());

        /* add walls */
        if (!is_flat_object)
        {
            const std::size_t N = road_object.outline.size();
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