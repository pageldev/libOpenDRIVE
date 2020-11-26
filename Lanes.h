#pragma once

#include "Geometries/CubicSpline.h"
#include "Math.hpp"
#include "Utils.hpp"

#include <map>
#include <memory>
#include <set>
#include <string>

namespace odr
{
class Road;
struct LaneSection;

struct Lane : public std::enable_shared_from_this<Lane>
{
    Lane(int id, bool level, std::string type);

    int  id;
    bool level = false;
    int  predecessor = 0;
    int  successor = 0;

    std::string type;
    CubicSpline lane_width;
};

using LaneSet = std::set<std::shared_ptr<Lane>, SharedPtrCmp<Lane, int, &Lane::id>>;

struct LaneSection : public std::enable_shared_from_this<LaneSection>
{
    LaneSection(double s0);
    LaneSet get_lanes();

    double s0;

    std::map<int, std::shared_ptr<Lane>> id_to_lane;
};

using LaneSectionSet = std::set<std::shared_ptr<LaneSection>, SharedPtrCmp<LaneSection, double, &LaneSection::s0>>;

} // namespace odr