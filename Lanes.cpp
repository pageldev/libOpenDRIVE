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

std::map<int, std::vector<Vec3D>> LaneSection::get_lane_outlines(double resolution) const
{
    if (auto road_ptr = this->road.lock())
    {
        auto s0_lanesec_iter = road_ptr->s0_to_lanesection.find(this->s0);
        if (s0_lanesec_iter == road_ptr->s0_to_lanesection.end())
            throw std::runtime_error("road associated with wrong lane section");

        const bool   is_last = (s0_lanesec_iter == std::prev(road_ptr->s0_to_lanesection.end()));
        const double next_s0 = is_last ? road_ptr->length : std::next(s0_lanesec_iter)->first;
        const double lanesec_len = next_s0 - this->s0;

        std::vector<double> s_vals;
        for (double s = this->s0; s < this->s0 + lanesec_len; s += resolution)
            s_vals.push_back(s);
        s_vals.push_back(next_s0 - (1e-9));

        std::map<int, std::vector<Vec3D>> lane_id_to_outer_brdr_line;
        std::map<int, std::vector<Vec3D>> lane_id_to_inner_brdr_line;
        for (const double& s : s_vals)
        {
            const std::map<int, double> lane_borders = road_ptr->get_lane_borders(s);
            for (auto id_brdr_iter = lane_borders.begin(); id_brdr_iter != lane_borders.end(); id_brdr_iter++)
            {
                const int lane_id = id_brdr_iter->first;
                if (lane_id == 0)
                    continue;

                const double t_outer_brdr = id_brdr_iter->second;
                const double t_inner_brdr = (lane_id > 0) ? std::prev(id_brdr_iter)->second : std::next(id_brdr_iter)->second;

                double h_inner_brdr = -std::tan(road_ptr->crossfall.get_crossfall(s, (lane_id > 0))) * t_inner_brdr;
                double h_outer_brdr = 0;
                if (this->id_to_lane.at(lane_id)->level)
                {
                    const double superelev = road_ptr->superelevation.get(s); // cancel out superelevation
                    h_outer_brdr = h_inner_brdr + std::tan(superelev) * (t_outer_brdr - t_inner_brdr);
                }
                else
                {
                    h_outer_brdr = -std::tan(road_ptr->crossfall.get_crossfall(s, (lane_id > 0))) * t_outer_brdr;
                }

                if (this->id_to_lane.at(lane_id)->s0_to_height_offset.size() > 0)
                {
                    const std::map<double, HeightOffset>& height_offs = this->id_to_lane.at(lane_id)->s0_to_height_offset;
                    auto                                  s0_height_offs_iter = height_offs.upper_bound(s - this->s0);
                    if (s0_height_offs_iter != height_offs.begin())
                        s0_height_offs_iter--;

                    h_inner_brdr += s0_height_offs_iter->second.inner;
                    h_outer_brdr += s0_height_offs_iter->second.outer;
                }

                lane_id_to_inner_brdr_line[lane_id].push_back(road_ptr->get_xyz(s, t_inner_brdr, h_inner_brdr));
                lane_id_to_outer_brdr_line[lane_id].push_back(road_ptr->get_xyz(s, t_outer_brdr, h_outer_brdr));
            }
        }

        std::map<int, std::vector<Vec3D>> lane_id_to_outline;
        for (const auto& id_lane : this->id_to_lane)
        {
            const int lane_id = id_lane.first;
            if (lane_id == 0)
                continue;

            const std::vector<Vec3D>& inner_brdr_line = lane_id_to_inner_brdr_line.at(lane_id);
            const std::vector<Vec3D>& outer_brdr_line = lane_id_to_outer_brdr_line.at(lane_id);

            std::vector<Vec3D> simplified_inner_brdr_line;
            rdp(inner_brdr_line, resolution, simplified_inner_brdr_line);

            std::vector<Vec3D> simplified_outer_brdr_line;
            rdp(outer_brdr_line, resolution, simplified_outer_brdr_line);

            lane_id_to_outline[lane_id] = std::vector<Vec3D>(simplified_inner_brdr_line.rbegin(), simplified_inner_brdr_line.rend());
            lane_id_to_outline.at(lane_id).insert(
                lane_id_to_outline.at(lane_id).end(), simplified_outer_brdr_line.begin(), simplified_outer_brdr_line.end());
            lane_id_to_outline.at(lane_id).push_back(simplified_inner_brdr_line.back());
        }

        return lane_id_to_outline;
    }
    else
    {
        throw std::runtime_error("could not access parent road for lane section");
    }

    return {};
}

std::vector<LaneVertices> LaneSection::get_lane_vertices(double resolution) const
{
    std::vector<LaneVertices>         lane_vertices;
    std::map<int, std::vector<Vec3D>> lane_id_to_outline = this->get_lane_outlines(resolution);
    for (const auto& id_outline : lane_id_to_outline)
    {
        const int                       lane_id = id_outline.first;
        std::vector<std::vector<Vec3D>> lane_outline{id_outline.second};
        std::vector<size_t>             indices = mapbox::earcut<size_t>(lane_outline);
        lane_vertices.push_back({id_outline.second, indices, lane_id, this->id_to_lane.at(lane_id)->type});
    }

    return lane_vertices;
}

} // namespace odr