#include "libodr/Lane.h"
#include "libodr/LaneSection.h"
#include "libodr/Mesh.h"
#include "libodr/Road.h"
#include "libodr/RoadMark.h"
#include "libodr/Sampler.h"
#include "libodr/Utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fmt/format.h>
#include <iterator>
#include <set>

namespace odr
{

HeightOffset::HeightOffset(double s_offset, double inner, double outer) : s_offset(s_offset), inner(inner), outer(outer)
{
    require_or_throw(s_offset >= 0, "s must be greater than or equal to 0 (got {})", s_offset);
    require_or_throw(!std::isnan(inner), "inner border must not be NaN");
    require_or_throw(!std::isnan(outer), "outer border must not be NaN");
}

LaneKey::LaneKey(const std::string& road_id, double lane_section_s, int lane_id) : road_id(road_id), lane_section_s(lane_section_s), lane_id(lane_id)
{
}

std::string LaneKey::to_string() const
{
    return fmt::format("{}/{:.17g}/{}", this->road_id, this->lane_section_s, this->lane_id);
}

Lane::Lane(int id, std::optional<std::string> type, std::optional<bool> level) : id(id), type(type), level(level) {}

Vec3D Lane::get_surface_pt(double s, double t, Vec3D* vn, bool allow_extrapolate) const
{
    const LaneSection& lane_section = *get_parent_or_throw<LaneSection>(*this);
    const Road&        road = *get_parent_or_throw<Road>(lane_section);

    const double lane_section_s_end = lane_section.get_end();
    require_or_throw(allow_extrapolate || (s >= lane_section.s && s <= lane_section_s_end),
                     "s must be in lane section range [{}, {}] (got {})",
                     lane_section.s,
                     lane_section_s_end,
                     s);
    const double s_road_clamped = std::min(std::max(s, 0.0), road.length);

    const Lane& inner_lane = lane_section.id_to_lane.at(next_towards_zero(this->id));

    const std::optional<double> t_inner_brdr_opt = inner_lane.outer_border.evaluate(s_road_clamped);
    require_or_throw(t_inner_brdr_opt.has_value() || inner_lane.id == 0, "lane {} has no outer border at s {}", inner_lane.id, s_road_clamped);
    const double t_inner_brdr = t_inner_brdr_opt.value_or(0.0);
    double       h = 0;

    // OpenDRIVE® Format Specification, Rev. 1.4, 5.3.7.2.1.1 Lane Record:
    // "keep lane on level, .i.e. do not apply superelevation or crossfall"
    if (this->level.value_or(false))
    {
        // compensate crossfall and superelevation to level lane
        const double alpha = road.crossfall.get(s_road_clamped, (this->id > 0));
        const double theta = road.superelevation.evaluate(s_road_clamped).value_or(0.0);
        h = -std::tan(alpha) * std::abs(t_inner_brdr) + std::tan(theta) * (t - t_inner_brdr);
    }
    else
    {
        h = -std::tan(road.crossfall.get(s_road_clamped, (this->id > 0))) * std::abs(t);
    }

    // OpenDRIVE® Format Specification, Rev. 1.4, 5.3.7.2.1.1.9 Lane Height Record:
    // "The surface of a lane may be offset from the plane defined by the reference line and the corresponding elevation and crossfall entries"
    if (!this->s_to_height_offset.empty())
    {
        const auto heights_iter = this->s_to_height_offset.upper_bound(s_road_clamped); // first element > s
        if (heights_iter != this->s_to_height_offset.begin())                           // s after first <height> record
        {
            const HeightOffset&         height_offset = std::prev(heights_iter)->second;
            const double                h_inner = height_offset.inner;
            const double                h_outer = height_offset.outer;
            const std::optional<double> t_outer_brdr = this->outer_border.evaluate(s_road_clamped);
            require_or_throw(t_outer_brdr.has_value() || this->id == 0, "lane {} has no outer border at s {}", this->id, s_road_clamped);
            const double t_outer_brdr_value = t_outer_brdr.value_or(0.0);
            const double t_norm = (t_outer_brdr_value != t_inner_brdr) ? (t - t_inner_brdr) / (t_outer_brdr_value - t_inner_brdr) : 0.0; // [0,1]
            h += t_norm * (h_outer - h_inner) + h_inner;
        }
    }

    return road.get_xyz(s, t, h, nullptr, nullptr, vn, allow_extrapolate);
}

Mesh3D Lane::get_mesh(double eps, std::vector<uint32_t>* outline_indices) const
{
    const LaneSection&     lane_section = *get_parent_or_throw<LaneSection>(*this);
    const LaneSampler      lane_sampler(*this);
    const std::set<double> samples = lane_sampler.get_mesh_s_samples(lane_section.s, lane_section.get_end(), eps);

    Mesh3D mesh;
    for (const double s : samples)
    {
        for (const Lane* edge_lane : {this, &lane_sampler.inner_lane})
        {
            const std::optional<double> t = edge_lane->outer_border.evaluate(s);
            require_or_throw(t.has_value() || edge_lane->id == 0, "lane {} has no outer border at s {}", edge_lane->id, s);
            Vec3D normal{0, 0, 0};
            mesh.vertices.push_back(this->get_surface_pt(s, t.value_or(0.0), &normal));
            mesh.normals.push_back(normal);
            mesh.st_coordinates.push_back({s, t.value_or(0.0)});
        }
    }

    const bool ccw = this->id < 0;
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

Mesh3D Lane::get_roadmark_mesh(const SingleRoadMark& roadmark, double eps, bool enforce_road_bounds) const
{
    if (is_zero(roadmark.width))
        return Mesh3D{};

    const std::set<double> s_samples = LaneSampler(*this).get_border_s_samples(roadmark.s_start, roadmark.s_end, roadmark.t_offset, eps);

    Mesh3D out_mesh;
    for (const double s : s_samples)
    {
        Vec3D                       vn_edge_a{0, 0, 0};
        const std::optional<double> t_lane_outer_border = this->outer_border.evaluate(s);
        require_or_throw(t_lane_outer_border.has_value() || this->id == 0, "lane {} has no outer border at s {}", this->id, s);
        const double t_edge_a = t_lane_outer_border.value_or(0.0) + roadmark.width * 0.5 + roadmark.t_offset;
        out_mesh.vertices.push_back(this->get_surface_pt(s, t_edge_a, &vn_edge_a, !enforce_road_bounds));
        out_mesh.normals.push_back(vn_edge_a);

        Vec3D        vn_edge_b{0, 0, 0};
        const double t_edge_b = t_edge_a - roadmark.width;
        out_mesh.vertices.push_back(this->get_surface_pt(s, t_edge_b, &vn_edge_b, !enforce_road_bounds));
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

std::vector<SingleRoadMark> Lane::get_roadmarks(double s_start, double s_end) const
{
    if ((s_start == s_end) || this->s_to_road_mark.empty())
        return {};

    // OpenDRIVE Format Specification, Rev. 1.8.1, 11.8 Road markings:
    // "The <roadMark> elements of a lane shall remain valid until another <roadMark> element starts or the lane section ends."
    auto rm_iter_start = this->s_to_road_mark.upper_bound(s_start); // first element > s
    if (rm_iter_start != this->s_to_road_mark.begin())
        rm_iter_start--;
    auto rm_iter_end = this->s_to_road_mark.lower_bound(s_end); // first element >= s

    std::vector<SingleRoadMark> roadmarks;
    for (auto rm_iter = rm_iter_start; rm_iter != rm_iter_end; rm_iter++)
    {
        const double    s_road_mark = rm_iter->first;
        const RoadMark& roadmark = rm_iter->second;

        double width = RoadMark::StandardWidth;
        if (roadmark.width)
            width = roadmark.width.value();
        else if (roadmark.weight.value_or("standard") == "bold")
            width = RoadMark::BoldWidth;

        const double s_end_road_mark = (std::next(rm_iter) == rm_iter_end) ? s_end : std::min(std::next(rm_iter)->first, s_end);

        if (roadmark.type_elem)
        {
            for (const RoadMarkLine& rm_line : roadmark.type_elem->lines)
            {
                if (is_zero(rm_line.length))
                    continue;

                width = rm_line.width.value_or(width);
                const double space = rm_line.space.value_or(0);

                const double s_road_mark_line = s_road_mark + rm_line.s_offset;
                for (double s_single_road_mark = s_road_mark_line; s_single_road_mark < s_end_road_mark;
                     s_single_road_mark += (rm_line.length + space))
                {
                    const double s_end_single_road_mark = std::min(s_end, s_single_road_mark + rm_line.length);
                    roadmarks.emplace_back(s_single_road_mark, s_end_single_road_mark, rm_line.t_offset, width, roadmark.type);
                    if (is_zero(space)) // treat roadMark::type::line with space = 0 as single roadmark
                        break;
                }
            }
        }
        else
        {
            roadmarks.emplace_back(std::max(s_road_mark, s_start), s_end_road_mark, 0, width, roadmark.type);
        }
    }

    return roadmarks;
}

} // namespace odr
