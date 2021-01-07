#pragma once

#include "Geometries/CubicSpline.h"
#include "Math.hpp"
#include "RoadMark.h"
#include "Utils.hpp"

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace odr
{
struct HeightOffset
{
    double inner;
    double outer;
};

struct Lane : public std::enable_shared_from_this<Lane>
{
    Lane(int id, bool level, std::string type);

    int  id;
    bool level = false;
    int  predecessor = 0;
    int  successor = 0;

    std::string type;
    CubicSpline lane_width;
    CubicSpline lane_border;

    std::map<double, HeightOffset> s0_to_height_offset;
    std::map<double, RoadMark>     s0_to_roadmark;
};

using ConstLaneSet = std::set<std::shared_ptr<const Lane>, SharedPtrCmp<const Lane, int, &Lane::id>>;
using LaneSet = std::set<std::shared_ptr<Lane>, SharedPtrCmp<Lane, int, &Lane::id>>;

} // namespace odr