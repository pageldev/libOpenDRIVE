#include "LaneSection.h"
#include "Road.h"

#include <iterator>
#include <stdexcept>
#include <utility>

namespace odr
{
LaneSection::LaneSection(double s0) : s0(s0) {}

double LaneSection::get_end() const
{
    if (auto road_ptr = this->road.lock())
    {
        auto s_lanesec_iter = road_ptr->s_to_lanesection.find(this->s0);
        if (s_lanesec_iter == road_ptr->s_to_lanesection.end())
            throw std::runtime_error("road associated with wrong lane section");

        const bool   is_last = (s_lanesec_iter == std::prev(road_ptr->s_to_lanesection.end()));
        const double next_s0 = is_last ? road_ptr->length : std::next(s_lanesec_iter)->first;

        return next_s0;
    }
    else
    {
        throw std::runtime_error("could not access parent road for lane section");
    }

    return 0;
}

ConstLaneSet LaneSection::get_lanes() const
{
    ConstLaneSet lanes;
    for (const auto& id_lane : this->id_to_lane)
        lanes.insert(id_lane.second);

    return lanes;
}

LaneSet LaneSection::get_lanes()
{
    LaneSet lanes;
    for (const auto& id_lane : this->id_to_lane)
        lanes.insert(id_lane.second);

    return lanes;
}

std::shared_ptr<const Lane> LaneSection::get_lane(double s, double t) const
{
    if (this->id_to_lane.at(0)->outer_border.get(s) == t) // exactly on lane #0
        return id_to_lane.at(0);

    std::map<double, int> outer_border_to_lane_id;
    for (const auto& id_lane : id_to_lane)
        outer_border_to_lane_id[id_lane.second->outer_border.get(s)] = id_lane.first;

    auto target_iter = outer_border_to_lane_id.lower_bound(t);
    if (target_iter == outer_border_to_lane_id.end()) // past upper boundary
        target_iter--;

    if (target_iter->second <= 0 && target_iter != outer_border_to_lane_id.begin() && t != target_iter->first)
        target_iter--;

    return this->id_to_lane.at(target_iter->second);
}

std::shared_ptr<Lane> LaneSection::get_lane(double s, double t)
{
    std::shared_ptr<Lane> lane = std::const_pointer_cast<Lane>(static_cast<const LaneSection&>(*this).get_lane(s, t));
    return lane;
}

std::map<int, std::pair<Line3D, Line3D>> LaneSection::get_lane_border_lines(double resolution) const
{
    if (auto road_ptr = this->road.lock())
    {
        const double s_end = this->get_end();

        std::vector<double> s_vals;
        for (double s = this->s0; s < s_end; s += resolution)
            s_vals.push_back(s);
        s_vals.push_back(s_end);

        std::map<int, std::pair<Line3D, Line3D>> lane_id_to_outer_inner_brdr_line;
        for (const double& s : s_vals)
        {
            for (const auto& id_lane : this->id_to_lane)
            {
                const int lane_id = id_lane.first;
                if (lane_id == 0)
                    continue;

                const double t_outer_brdr = id_lane.second->outer_border.get(s);
                const double t_inner_brdr = id_lane.second->inner_border.get(s);

                lane_id_to_outer_inner_brdr_line[lane_id].first.push_back(road_ptr->get_surface_pt(s, t_outer_brdr));
                lane_id_to_outer_inner_brdr_line[lane_id].second.push_back(road_ptr->get_surface_pt(s, t_inner_brdr));
            }
        }

        return lane_id_to_outer_inner_brdr_line;
    }
    else
    {
        throw std::runtime_error("could not access parent road for lane section");
    }

    return {};
}

std::vector<LaneVertices> LaneSection::get_lane_vertices(double resolution) const
{
    std::vector<LaneVertices>                lanesection_vertices;
    std::map<int, std::pair<Line3D, Line3D>> lane_id_to_outer_inner_brdr_line = this->get_lane_border_lines(resolution);

    for (auto& id_outer_inner_brdr_line : lane_id_to_outer_inner_brdr_line)
    {
        const int    lane_id = id_outer_inner_brdr_line.first;
        LaneVertices lane_vertices;
        lane_vertices.lane_id = lane_id;
        lane_vertices.type = this->id_to_lane.at(lane_id)->type;

        Line3D& outer_brdr_line = id_outer_inner_brdr_line.second.first;
        Line3D& inner_brdr_line = id_outer_inner_brdr_line.second.second;
        if (outer_brdr_line.size() != inner_brdr_line.size())
            throw std::runtime_error("outer and inner border line should have equal number of points");

        lane_vertices.vertices = outer_brdr_line;
        lane_vertices.vertices.insert(lane_vertices.vertices.end(), inner_brdr_line.rbegin(), inner_brdr_line.rend());

        const size_t num_pts = lane_vertices.vertices.size();
        for (size_t l_idx = 1, r_idx = num_pts - 2; l_idx < (num_pts >> 1); l_idx++, r_idx--)
        {
            std::vector<size_t> indicies_patch;
            if (lane_id > 0) // make sure triangle normal is facing "up"
                indicies_patch = {l_idx, l_idx - 1, r_idx + 1, r_idx, l_idx, r_idx + 1};
            else
                indicies_patch = {l_idx, r_idx + 1, l_idx - 1, r_idx, r_idx + 1, l_idx};
            lane_vertices.indices.insert(lane_vertices.indices.end(), indicies_patch.begin(), indicies_patch.end());
        }

        lanesection_vertices.push_back(lane_vertices);
    }

    return lanesection_vertices;
}

std::vector<RoadMarkPolygon> LaneSection::get_roadmark_polygons(int lane_id, double resolution) const
{
    if (auto road_ptr = this->road.lock())
    {
        std::vector<RoadMarkPolygon> roadmark_polygons;

        const double s_end = this->get_end();
        const auto&  lane = this->id_to_lane.at(lane_id);
        for (auto s_roadmark_iter = lane->s_to_roadmark.begin(); s_roadmark_iter != lane->s_to_roadmark.end(); s_roadmark_iter++)
        {
            const bool   is_last = (s_roadmark_iter == std::prev(lane->s_to_roadmark.end()));
            const double next_s = is_last ? s_end : std::next(s_roadmark_iter)->first;

            const RoadMark& roadmark = s_roadmark_iter->second;
            if (roadmark.lines.empty())
            {
                std::vector<double> s_vals;
                for (double s = s_roadmark_iter->first; s < next_s; s += resolution)
                    s_vals.push_back(s);
                s_vals.push_back(next_s);

                RoadMarkPolygon roadmark_polygon;
                const double    width = roadmark.width > 0 ? roadmark.width : 0.2;
                for (const double& s : s_vals)
                    roadmark_polygon.outline.push_back(road_ptr->get_surface_pt(s, lane->outer_border.get(s) - 0.5 * width));
                for (auto r_iter = s_vals.rbegin(); r_iter != s_vals.rend(); r_iter++)
                    roadmark_polygon.outline.push_back(road_ptr->get_surface_pt(*r_iter, lane->outer_border.get(*r_iter) + 0.5 * width));
                roadmark_polygons.push_back(roadmark_polygon);
            }
            else
            {
                for (const RoadMarkLine& roadmark_line : roadmark.lines)
                {
                    for (double s_start_roadmark = s_roadmark_iter->first + roadmark_line.sOffset; s_start_roadmark < next_s;
                         s_start_roadmark += roadmark_line.length)
                    {
                        const double        s_end_roadmark = s_start_roadmark + (roadmark_line.length - roadmark_line.space);
                        std::vector<double> s_vals;
                        for (double s = s_start_roadmark; s < s_end_roadmark; s += resolution)
                            s_vals.push_back(s);
                        s_vals.push_back(s_end_roadmark);

                        RoadMarkPolygon roadmark_polygon;
                        const double    width = roadmark_line.width > 0 ? roadmark_line.width : 0.2;
                        for (const double& s : s_vals)
                            roadmark_polygon.outline.push_back(road_ptr->get_surface_pt(s, lane->outer_border.get(s) - 0.5 * width));
                        for (auto r_iter = s_vals.rbegin(); r_iter != s_vals.rend(); r_iter++)
                            roadmark_polygon.outline.push_back(road_ptr->get_surface_pt(*r_iter, lane->outer_border.get(*r_iter) + 0.5 * width));
                        roadmark_polygons.push_back(roadmark_polygon);
                    }
                }
            }
        }

        return roadmark_polygons;
    }
    else
    {
        throw std::runtime_error("could not access parent road for lane section");
    }

    return {};
}

std::vector<RoadMarkPolygon> LaneSection::get_roadmark_polygons(double resolution) const
{
    std::vector<RoadMarkPolygon> roadmark_polygons;
    for (const auto& id_lane : this->id_to_lane)
    {
        std::vector<RoadMarkPolygon> lane_roadmarks = this->get_roadmark_polygons(id_lane.first, resolution);
        roadmark_polygons.insert(roadmark_polygons.end(), lane_roadmarks.begin(), lane_roadmarks.end());
    }

    return roadmark_polygons;
}

} // namespace odr