#pragma once
#include "libodr/Lane.h"

#include <map>
#include <string>

namespace odr
{

struct LaneSection
{
    LaneSection(double s);

    // if t falls on a lane boundary, the inner lane (closer to lane #0) is returned
    int get_lane_id(double s, double t) const;

    double s;

    std::map<int, Lane> id_to_lane;
};

} // namespace odr
