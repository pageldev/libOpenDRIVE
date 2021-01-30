#include "LaneSection.h"
#include "RefLine.h"
#include "Road.h"

#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

namespace odr
{
LaneSection::LaneSection(double s0) : s0(s0) {}

double LaneSection::get_end() const
{
    auto road_ptr = this->road.lock();
    if (!road_ptr)
        throw std::runtime_error("could not access parent road for lane section");

    auto s_lanesec_iter = road_ptr->s_to_lanesection.find(this->s0);
    if (s_lanesec_iter == road_ptr->s_to_lanesection.end())
        throw std::runtime_error("road associated with wrong lane section");

    const bool   is_last = (s_lanesec_iter == std::prev(road_ptr->s_to_lanesection.end()));
    const double next_s0 = is_last ? road_ptr->length : std::next(s_lanesec_iter)->first;

    return next_s0 - std::numeric_limits<double>::min();
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

} // namespace odr