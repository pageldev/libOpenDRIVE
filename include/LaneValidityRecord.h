#pragma once
#include "XmlNode.h"

#include <climits>

namespace odr
{

struct LaneValidityRecord : public XmlNode
{
    LaneValidityRecord(int from_lane, int to_lane) : from_lane(from_lane), to_lane(to_lane) {}

    int from_lane = INT_MIN;
    int to_lane = INT_MAX;
};

} // namespace odr
