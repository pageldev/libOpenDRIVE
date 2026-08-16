#pragma once
#include <climits>

namespace odr
{

struct LaneValidity
{
    LaneValidity(int from_lane, int to_lane) : from_lane(from_lane), to_lane(to_lane) {}

    int from_lane;
    int to_lane;
};

} // namespace odr
