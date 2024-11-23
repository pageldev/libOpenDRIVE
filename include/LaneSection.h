#pragma once
#include "Lane.h"
#include "XmlNode.h"

#include <map>
#include <string>
#include <vector>

namespace odr
{

struct LaneSection : public XmlNode
{
    LaneSection(std::string road_id, double s0);

    std::vector<Lane> get_lanes() const;

    // if t falls on a lane boundary, the inner lane (closer to lane #0) is returned
    int  get_lane_id(const double s, const double t) const;
    Lane get_lane(const int id) const;
    Lane get_lane(const double s, const double t) const;

    std::string         road_id = "";
    double              s0 = 0;
    std::map<int, Lane> id_to_lane;
};

} // namespace odr
