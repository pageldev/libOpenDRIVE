#include "Lanes.h"
#include "RefLine.h"
#include "Road.h"
#include "Utils.hpp"

namespace odr
{
Lane::Lane(int id, bool level, std::string type) : id(id), level(level), type(type) {}

Vec3D Lane::get_surface_pt(double s, double t, Vec3D* vn) const
{
    auto road_ptr = this->road.lock();
    if (!road_ptr)
        throw std::runtime_error("could not access parent road for lane section");

    const double t_inner_brdr = this->inner_border.get(s);
    double       h_t = 0;

    if (this->level)
    {
        const double h_inner_brdr = -std::tan(road_ptr->crossfall.get_crossfall(s, (this->id > 0))) * std::abs(t_inner_brdr);
        const double superelev = road_ptr->superelevation.get(s); // cancel out superelevation
        h_t = h_inner_brdr + std::tan(superelev) * (t - t_inner_brdr);
    }
    else
    {
        h_t = -std::tan(road_ptr->crossfall.get_crossfall(s, (this->id > 0))) * std::abs(t);
    }

    if (this->s_to_height_offset.size() > 0)
    {
        const std::map<double, HeightOffset>& height_offs = this->s_to_height_offset;

        auto s0_height_offs_iter = height_offs.upper_bound(s);
        if (s0_height_offs_iter != height_offs.begin())
            s0_height_offs_iter--;

        const double t_outer_brdr = this->outer_border.get(s);
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

    return road_ptr->get_xyz(s, t, h_t, nullptr, nullptr, vn);
}

std::set<double> Lane::approximate_border_linear(double s_start, double s_end, double eps, bool outer) const
{
    auto road_ptr = this->road.lock();
    if (!road_ptr)
        throw std::runtime_error("could not access parent road for lane section");

    std::set<double> s_vals = road_ptr->ref_line->approximate_linear(eps, s_start, s_end);

    const CubicSpline& border = outer ? this->outer_border : this->inner_border;
    std::set<double>   s_vals_brdr = border.approximate_linear(eps, s_start, s_end);
    s_vals.insert(s_vals_brdr.begin(), s_vals_brdr.end());

    std::set<double> s_vals_lane_height = extract_keys(this->s_to_height_offset);
    s_vals.insert(s_vals_lane_height.begin(), s_vals_lane_height.end());

    const double     t_max = this->outer_border.get_max(s_start, s_end);
    std::set<double> s_vals_superelev = road_ptr->superelevation.approximate_linear(std::atan(eps / std::abs(t_max)), s_start, s_end);
    s_vals.insert(s_vals_superelev.begin(), s_vals_superelev.end());

    return s_vals;
}

Line3D Lane::get_border_line(double s_start, double s_end, double eps, bool outer) const
{
    auto road_ptr = this->road.lock();
    if (!road_ptr)
        throw std::runtime_error("could not access parent road for lane section");

    std::set<double> s_vals = this->approximate_border_linear(s_start, s_end, eps, outer);

    Line3D border_line;
    for (const double& s : s_vals)
    {
        const double t = outer ? this->outer_border.get(s) : this->inner_border.get(s);
        border_line.push_back(this->get_surface_pt(s, t));
    }

    return border_line;
}

Mesh3D Lane::get_mesh(double s_start, double s_end, double eps, std::vector<uint32_t>* outline_indices) const
{
    auto road_ptr = this->road.lock();
    if (!road_ptr)
        throw std::runtime_error("could not access parent road for lane section");

    std::set<double> s_vals = road_ptr->ref_line->approximate_linear(eps, s_start, s_end);
    std::set<double> s_vals_outer_brdr = this->outer_border.approximate_linear(eps, s_start, s_end);
    s_vals.insert(s_vals_outer_brdr.begin(), s_vals_outer_brdr.end());
    std::set<double> s_vals_inner_brdr = this->inner_border.approximate_linear(eps, s_start, s_end);
    s_vals.insert(s_vals_inner_brdr.begin(), s_vals_inner_brdr.end());

    std::set<double> s_vals_lane_height = extract_keys(this->s_to_height_offset);
    s_vals.insert(s_vals_lane_height.begin(), s_vals_lane_height.end());

    const double     t_max = this->outer_border.get_max(s_start, s_end);
    std::set<double> s_vals_superelev = road_ptr->superelevation.approximate_linear(std::atan(eps / std::abs(t_max)), s_start, s_end);
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
        const double t_inner_brdr = this->inner_border.get(s);
        out_mesh.vertices.push_back(this->get_surface_pt(s, t_inner_brdr, &vn_inner_brdr));
        out_mesh.normals.push_back(vn_inner_brdr);
        out_mesh.st_coordinates.push_back({s, t_inner_brdr});

        Vec3D        vn_outer_brdr{0, 0, 0};
        const double t_outer_brdr = this->outer_border.get(s);
        out_mesh.vertices.push_back(this->get_surface_pt(s, t_outer_brdr, &vn_outer_brdr));
        out_mesh.normals.push_back(vn_outer_brdr);
        out_mesh.st_coordinates.push_back({s, t_outer_brdr});
    }

    const size_t num_pts = out_mesh.vertices.size();
    const bool   ccw = this->id > 0;
    for (size_t idx = 3; idx < num_pts; idx += 2)
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

std::vector<std::shared_ptr<RoadMark>> Lane::get_roadmarks(double s_start, double s_end) const
{
    if ((s_start == s_end) || this->s_to_roadmark_group.empty())
        return {};

    auto s_end_rm_iter = this->s_to_roadmark_group.lower_bound(s_end);
    auto s_start_rm_iter = this->s_to_roadmark_group.upper_bound(s_start);
    if (s_start_rm_iter != this->s_to_roadmark_group.begin())
        s_start_rm_iter--;

    std::vector<std::shared_ptr<RoadMark>> roadmarks;
    for (auto s_rm_iter = s_start_rm_iter; s_rm_iter != s_end_rm_iter; s_rm_iter++)
    {
        const RoadMarkGroup& roadmark_group = s_rm_iter->second;

        const double s_start_roadmark_group = std::max(s_rm_iter->first, s_start);
        const double s_end_roadmark_group = (std::next(s_rm_iter) == s_end_rm_iter) ? s_end : std::min(std::next(s_rm_iter)->first, s_end);

        double width = roadmark_group.weight == "bold" ? ROADMARK_WEIGHT_BOLD_WIDTH : ROADMARK_WEIGHT_STANDARD_WIDTH;
        if (roadmark_group.s_to_roadmarks_line.size() == 0)
        {
            if (roadmark_group.width > 0)
                width = roadmark_group.width;

            roadmarks.push_back(std::make_shared<RoadMark>(RoadMark{s_start_roadmark_group, s_end_roadmark_group, 0, width, roadmark_group.type}));
        }
        else
        {
            for (const auto& s_roadmarks_line : roadmark_group.s_to_roadmarks_line)
            {
                const RoadMarksLine& roadmarks_line = s_roadmarks_line.second;
                if (roadmarks_line.width > 0)
                    width = roadmarks_line.width;

                if ((roadmarks_line.length + roadmarks_line.space) == 0)
                    continue;

                for (double s_start_single_roadmark = s_roadmarks_line.first; s_start_single_roadmark < s_end_roadmark_group;
                     s_start_single_roadmark += (roadmarks_line.length + roadmarks_line.space))
                {
                    const double s_end_single_roadmark = std::min(s_end, s_start_single_roadmark + roadmarks_line.length);
                    roadmarks.push_back(std::make_shared<RoadMark>(RoadMark{
                        s_start_single_roadmark, s_end_single_roadmark, roadmarks_line.t_offset, width, roadmark_group.type + roadmarks_line.name}));
                }
            }
        }
    }

    return roadmarks;
}

Mesh3D Lane::get_roadmark_mesh(std::shared_ptr<const RoadMark> roadmark, double eps) const
{
    const std::set<double> s_vals = this->approximate_border_linear(roadmark->s_start, roadmark->s_end, eps, true);

    Mesh3D out_mesh;
    for (const double& s : s_vals)
    {
        Vec3D        vn_edge_a{0, 0, 0};
        const double t_edge_a = this->outer_border.get(s) + roadmark->width * 0.5 + roadmark->t_offset;
        out_mesh.vertices.push_back(this->get_surface_pt(s, t_edge_a, &vn_edge_a));
        out_mesh.normals.push_back(vn_edge_a);

        Vec3D        vn_edge_b{0, 0, 0};
        const double t_edge_b = t_edge_a - roadmark->width;
        out_mesh.vertices.push_back(this->get_surface_pt(s, t_edge_b, &vn_edge_b));
        out_mesh.normals.push_back(vn_edge_b);
    }

    const size_t num_pts = out_mesh.vertices.size();
    for (size_t idx = 3; idx < num_pts; idx += 2)
    {
        std::array<size_t, 6> indicies_patch = {idx - 3, idx, idx - 1, idx - 3, idx - 2, idx};
        out_mesh.indices.insert(out_mesh.indices.end(), indicies_patch.begin(), indicies_patch.end());
    }

    return out_mesh;
}

} // namespace odr