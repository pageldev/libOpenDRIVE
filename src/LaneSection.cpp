#include "libodr/LaneSection.h"
#include "libodr/Geometries/CubicSpline.h"
#include "libodr/Utils.hpp"

#include <optional>
#include <utility>

namespace odr
{
LaneSection::LaneSection(double s0) : s0(s0)
{
    require_or_throw(s0 >= 0, "s must be greater than or equal to 0 (got {})", s0);
}

std::vector<Lane> LaneSection::get_lanes() const
{
    return get_map_values(this->id_to_lane);
}

int LaneSection::get_lane_id(double s, double t) const
{
    // default to 0 so lane #0 is at t=0 if no lane offset is defined
    if (this->id_to_lane.at(0).outer_border.evaluate(s).value_or(0.0) == t) // exactly on lane #0
        return 0;

    std::map<double /*t*/, int /*id*/> outer_border_to_lane_id;
    for (const auto& [id, lane] : this->id_to_lane)
    {
        const std::optional<double> outer_brdr_t = lane.outer_border.evaluate(s);
        require_or_throw(outer_brdr_t.has_value() || id == 0, "lane {} has no outer border at s {}", id, s);
        outer_border_to_lane_id.insert({outer_brdr_t.value_or(0.0), id});
    }

    // lower_bound: first element >= t or past-the-end iterator if none is found
    auto target_iter = outer_border_to_lane_id.lower_bound(t);
    if (target_iter == outer_border_to_lane_id.end())
        target_iter--; // past outermost boundary -> select outermost lane

    // for t-negative lanes, lower_bound returns the neighboring inner lane (closer to lane ID 0); fix here
    if (target_iter->second <= 0 && target_iter != outer_border_to_lane_id.begin() && t != target_iter->first)
        target_iter--;

    return target_iter->second;
}

Lane LaneSection::get_lane(int id) const
{
    return this->id_to_lane.at(id);
}

Lane LaneSection::get_lane(double s, double t) const
{
    return this->id_to_lane.at(this->get_lane_id(s, t));
}

} // namespace odr