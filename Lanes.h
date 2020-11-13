#pragma once

#include "Road.h"

#include <map>
#include <memory>

namespace odr
{

struct Lane;

struct LaneSection : public std::enable_shared_from_this<LaneSection>
{
    LaneSection(double s0);

    double                               s0;
    std::shared_ptr<Road>                road;
    std::map<int, std::shared_ptr<Lane>> lanes;
};

struct Lane : public std::enable_shared_from_this<Lane>
{
    Lane(int id, std::string type);
    Vec3D get_outer_border_pt(double s) const;

    int                          id;
    std::string                  type;
    std::shared_ptr<LaneSection> lane_section;
    CubicSpline                  lane_width;
};

} // namespace odr