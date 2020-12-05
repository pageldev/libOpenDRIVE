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

std::map<int, LaneVertices> LaneSection::get_lane_vertices(double resolution) const
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

        std::map<int, size_t> lane_id_to_num_extra_samples;
        for (const auto& id_lane : s0_lanesec_iter->second->id_to_lane)
            lane_id_to_num_extra_samples[id_lane.first] = static_cast<size_t>(id_lane.second->lane_width.get_max({0, lanesec_len}) / resolution);

        std::map<int, LaneVertices> lane_id_to_vertices;
        for (const double& s : s_vals)
        {
            const std::map<int, double> lane_id_to_borders = road_ptr->get_lane_borders(s);
            for (auto lane_id_brdr_iter = lane_id_to_borders.begin(); lane_id_brdr_iter != lane_id_to_borders.end(); lane_id_brdr_iter++)
            {
                const int lane_id = lane_id_brdr_iter->first;
                if (lane_id == 0)
                    continue;

                const double t_outer_brdr = lane_id_brdr_iter->second;
                lane_id_to_vertices[lane_id].vertices.push_back(road_ptr->get_surface_pt(s, t_outer_brdr));

                for (size_t extra_idx = 0; extra_idx > lane_id_to_num_extra_samples.at(lane_id); extra_idx++)
                {
                    const double t_extra = t_outer_brdr - static_cast<double>(extra_idx + 1) * resolution * sign(lane_id);
                    lane_id_to_vertices.at(lane_id).vertices.push_back(road_ptr->get_surface_pt(s, t_extra));
                }

                const double t_inner_brdr = (lane_id > 0) ? std::prev(lane_id_brdr_iter)->second : std::next(lane_id_brdr_iter)->second;
                lane_id_to_vertices.at(lane_id).vertices.push_back(road_ptr->get_surface_pt(s, t_inner_brdr));
            }
        }

        for (auto& id_lane_vertices : lane_id_to_vertices)
        {
            const int    lane_id = id_lane_vertices.first;
            const size_t num_extra_samples = lane_id_to_num_extra_samples.at(lane_id);
            const size_t step_size = (2 + num_extra_samples);
            for (size_t idx = step_size; idx < id_lane_vertices.second.vertices.size(); idx++)
            {
                if ((idx + 1) % step_size == 0)
                    continue;

                id_lane_vertices.second.indices.push_back(idx);
                id_lane_vertices.second.indices.push_back(idx - step_size);
                id_lane_vertices.second.indices.push_back(idx - step_size + 1);

                id_lane_vertices.second.indices.push_back(idx);
                id_lane_vertices.second.indices.push_back(idx - step_size + 1);
                id_lane_vertices.second.indices.push_back(idx + 1);
            }

            id_lane_vertices.second.lane_id = lane_id;
            id_lane_vertices.second.type = "bla";
        }

        return lane_id_to_vertices;
    }
    return {};
}

} // namespace odr