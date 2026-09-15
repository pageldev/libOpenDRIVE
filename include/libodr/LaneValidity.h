#pragma once
#include "libodr/OdrNode.h"

namespace odr
{

struct LaneValidity : public OdrNode
{
    LaneValidity(int from_lane, int to_lane) : from_lane(from_lane), to_lane(to_lane) {}

    int from_lane;
    int to_lane;
};

} // namespace odr
