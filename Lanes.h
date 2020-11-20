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
struct Lane;
struct LaneSection;

struct Lane : public std::enable_shared_from_this<Lane>
{
    Lane(int id, std::string type);
    Vec3D get_outer_border_pt(double s) const;

    int         id;
    std::string type;
    CubicSpline lane_width;

    std::shared_ptr<LaneSection> lane_section;
};

using LaneSet = std::set<std::shared_ptr<Lane>, SharedPtrCmp<Lane, int, &Lane::id>>;

struct LaneSection : public std::enable_shared_from_this<LaneSection>
{
    LaneSection(double s0);
    LaneSet               get_lanes();
    std::map<int, double> get_lane_borders(double s) const;

    double                s0;
    std::shared_ptr<Road> road;

    std::map<int, std::shared_ptr<Lane>> id_to_lane;
};

using LaneSectionSet = std::set<std::shared_ptr<LaneSection>, SharedPtrCmp<LaneSection, double, &LaneSection::s0>>;

} // namespace odr