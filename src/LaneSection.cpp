#include "libodr/LaneSection.h"
#include "libodr/Geometries/CubicSpline.h"
#include "libodr/Road.h"
#include "libodr/Utils.hpp"

#include <cmath>
#include <iterator>
#include <limits>
#include <optional>
#include <utility>

namespace odr
{
LaneSection::LaneSection(double s) : s(s)
{
    require_or_throw(s >= 0, "s must be greater than or equal to 0 (got {})", s);
}

double LaneSection::get_end() const
{
    const Road& road = *get_parent_or_throw<Road>(*this);

    auto lane_section_iter = road.s_to_lane_section.find(this->s);
    require_or_throw(lane_section_iter != road.s_to_lane_section.end(), "no lane section found for s {}", this->s);

    const bool is_last = (lane_section_iter == std::prev(road.s_to_lane_section.end()));
    if (is_last)
        return road.length;

    const double s_next = std::next(lane_section_iter)->first;
    return std::nextafter(s_next, -std::numeric_limits<double>::infinity()); // to be within lane section
}

double LaneSection::get_length() const
{
    return this->get_end() - this->s;
}

int LaneSection::get_lane_id(double s, double t) const
{
    // default to 0 so lane #0 is at t=0 if no lane offset is defined
    if (this->id_to_lane.at(0).outer_border.evaluate(s).value_or(0.0) == t) // exactly on lane #0
        return 0;

    std::map<double /*t*/, int /*id*/> t_outer_border_to_lane_id;
    for (const auto& [id, lane] : this->id_to_lane)
    {
        const std::optional<double> t_outer_brdr = lane.outer_border.evaluate(s);
        require_or_throw(t_outer_brdr.has_value() || id == 0, "lane {} has no outer border at s {}", id, s);
        t_outer_border_to_lane_id.insert({t_outer_brdr.value_or(0.0), id});
    }

    // lower_bound: first element >= t or past-the-end iterator if none is found
    auto target_iter = t_outer_border_to_lane_id.lower_bound(t);
    if (target_iter == t_outer_border_to_lane_id.end())
        target_iter--; // past outermost boundary -> select outermost lane

    // for t-negative lanes, lower_bound returns the neighboring inner lane (closer to lane ID 0); fix here
    if (target_iter->second <= 0 && target_iter != t_outer_border_to_lane_id.begin() && t != target_iter->first)
        target_iter--;

    return target_iter->second;
}

} // namespace odr
