#include "LaneSection.h"
#include "Geometries/CubicSpline.h"
#include "Utils.hpp"

#include <utility>

namespace odr
{
LaneSection::LaneSection(std::string road_id, double s0) : road_id(road_id), s0(s0) {}

std::vector<Lane> LaneSection::get_lanes() const
{
    return get_map_values(this->id_to_lane);
}

int LaneSection::get_lane_id(const double s, const double t) const
{
    if (this->id_to_lane.at(0).outer_border.get(s) == t) // exactly on lane #0
        return 0;

    std::map<double /*t*/, int /*id*/> outer_border_to_lane_id;
    for (const auto& id_lane : id_to_lane)
    {
        const double outer_brdr_t = id_lane.second.outer_border.get(s);
        outer_border_to_lane_id.insert({outer_brdr_t, id_lane.first});
    }

    // std::map::lower_bound Return value:
    // Iterator pointing to the first element that is not less than (i.e. greater or equal to) key.
    // If no such element is found, a past-the-end iterator (see end()) is returned.

    // For t-positive lanes, lower_bound returns map::end when t exceeds the outermost lane boundary; fix here
    auto target_iter = outer_border_to_lane_id.lower_bound(t);
    if (target_iter == outer_border_to_lane_id.end())
        target_iter--; // past outermost boundary -> select outermost lane

    // For t-negative lanes, lower_bound returns the neighboring inner lane (closer to lane ID 0); fix here
    if (target_iter->second <= 0 && target_iter != outer_border_to_lane_id.begin() && t != target_iter->first)
        target_iter--;

    return target_iter->second;
}

Lane LaneSection::get_lane(const int id) const
{
    return this->id_to_lane.at(id);
}

Lane LaneSection::get_lane(const double s, const double t) const
{
    return this->id_to_lane.at(this->get_lane_id(s, t));
}

} // namespace odr