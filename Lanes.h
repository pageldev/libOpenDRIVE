#pragma once

#include "Road.h"

#include <map>
#include <memory>

namespace odr
{

struct LaneOffset
{
    LaneOffset(double s0, double a, double b, double c, double d);
    double get_offset(const double s) const;

    double s0, a, b, c, d;
};

struct LaneWidth
{
    LaneWidth(double sOffset, double a, double b, double c, double d);
    double get_width(const double ds) const;

    double s_offset, a, b, c, d;
};

struct Lane;

struct LaneSection : public std::enable_shared_from_this<LaneSection>
{
    LaneSection(double s0);
    void add_lane(std::shared_ptr<Lane> lane);

    double                               s0;
    std::shared_ptr<Road>                road;
    std::map<int, std::shared_ptr<Lane>> lanes;
};

struct Lane : public std::enable_shared_from_this<Lane>
{
    Lane(int id, std::string type, std::map<double, std::shared_ptr<LaneWidth>> lane_widths);
    Point3D<double> get_outer_border_pt(const double s) const;

    int                                          id;
    std::string                                  type;
    std::shared_ptr<LaneSection>                 lane_section;
    std::map<double, std::shared_ptr<LaneWidth>> lane_widths;
};

} // namespace odr