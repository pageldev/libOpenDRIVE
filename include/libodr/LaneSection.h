#pragma once
#include "libodr/Lane.h"

#include <map>
#include <string>
#include <vector>

namespace odr
{

struct LaneSection
{
    LaneSection(double s);

    std::vector<Lane> get_lanes() const;

    // if t falls on a lane boundary, the inner lane (closer to lane #0) is returned
    int  get_lane_id(double s, double t) const;
    Lane get_lane(int id) const;
    Lane get_lane(double s, double t) const;

    double s;

    std::map<int, Lane> id_to_lane;
};

} // namespace odr
