#include "Lanes.h"
#include "Road.h"

#include "earcut/earcut.hpp"

#include <iterator>
#include <stdexcept>
#include <utility>

namespace odr
{
Lane::Lane(int id, bool level, std::string type) : id(id), level(level), type(type) {}

LaneSection::LaneSection(double s0) : s0(s0) {}

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

std::vector<LaneVertices> LaneSection::get_lane_vertices(double resolution) const
{
    if (auto road_ptr = this->road.lock())
    {
        auto s0_lanesec_iter = road_ptr->s0_to_lanesection.find(this->s0);
        if (s0_lanesec_iter == road_ptr->s0_to_lanesection.end())
            throw std::runtime_error("road associated with wrong lane section");

        const bool   is_last = (s0_lanesec_iter == std::prev(road_ptr->s0_to_lanesection.end()));
        const double next_s0 = is_last ? road_ptr->length : std::next(s0_lanesec_iter)->first;
        const double lanesec_len = next_s0 - this->s0;

        const size_t num_s_vals = static_cast<size_t>(lanesec_len / resolution) + 1;
        const size_t num_lanes = this->id_to_lane.size();
        const size_t num_samples = num_lanes * num_s_vals;

        std::vector<double> s_vals;
        for (double s = this->s0; s < this->s0 + lanesec_len; s += resolution)
            s_vals.push_back(s);
        s_vals.push_back(next_s0 - (1e-9));

        /*
         * first store lane border points interleaved, e.g.
         *  p0   p1   p3  p4     outer border pts lane #1 at
         *  | -2 | -1 | 0 |   ->     start idx = 2
         *  p5   p6   p7  p8         using step = 4
         */
        std::vector<Vec3D> all_lane_outer_brdr_pts;
        all_lane_outer_brdr_pts.reserve(num_samples);
        for (const double& s : s_vals)
        {
            std::map<int, double> lane_borders = road_ptr->get_lane_borders(s);
            if (lane_borders.size() != num_lanes)
                throw std::runtime_error("unexpected number of lanes");
            for (const auto& id_t_brdr : lane_borders)
                all_lane_outer_brdr_pts.push_back(road_ptr->get_surface_pt(s, id_t_brdr.second));
        }

        /* extract and simplify lane border lines */
        std::map<int, std::vector<Vec3D>> lane_outer_border_line;
        for (size_t start_idx = 0; start_idx < num_lanes; start_idx++)
        {
            std::vector<Vec3D> simplified_outer_lane_border_pts;
            rdp(all_lane_outer_brdr_pts, resolution, simplified_outer_lane_border_pts, start_idx, num_lanes);
            const int lane_id = std::next(this->id_to_lane.begin(), start_idx)->first;
            lane_outer_border_line[lane_id] = simplified_outer_lane_border_pts;
        }

        /* assemble meshes */
        std::vector<LaneVertices> lane_vertices;
        for (auto id_pts_iter = lane_outer_border_line.begin(); id_pts_iter != lane_outer_border_line.end(); id_pts_iter++)
        {
            const int id = id_pts_iter->first;
            if (id == 0)
                continue;

            std::vector<std::vector<Vec3D>> lane_outline;
            lane_outline.push_back(std::vector<Vec3D>(id_pts_iter->second.rbegin(), id_pts_iter->second.rend()));
            if (id < 0)
                lane_outline.at(0).insert(lane_outline.at(0).end(), std::next(id_pts_iter)->second.begin(), std::next(id_pts_iter)->second.end());
            else
                lane_outline.at(0).insert(lane_outline.at(0).end(), std::prev(id_pts_iter)->second.begin(), std::prev(id_pts_iter)->second.end());
            lane_outline.at(0).push_back(lane_outline.at(0).front());

            std::vector<size_t> indices = mapbox::earcut<size_t>(lane_outline);
            lane_vertices.push_back({lane_outline.at(0), indices, id, this->id_to_lane.at(id)->type});
        }
        return lane_vertices;
    }
    return {};
}

} // namespace odr