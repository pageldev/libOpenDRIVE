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

    int  get_lane_id(double s, double t) const;
    Lane get_lane(double s, double t) const;

    std::string         road_id = "";
    double              s0 = 0;
    std::map<int, Lane> id_to_lane;
};

} // namespace odr
