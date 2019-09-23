#pragma once

#include "Road.h"

#include <map>
#include <memory>
#include <vector>


class Road;
struct LaneSection;

struct LaneWidth
{
    LaneWidth(double sOffset, double a, double b, double c, double d);
    double get_width(double ds);

    double s_offset, a, b, c, d;
};


struct Lane
{
    Lane(int id, std::vector<std::shared_ptr<LaneWidth>> lane_widths);
    std::pair<double, double> get_outer_border_pt(double s);

    int id;
    std::shared_ptr<LaneSection> lanesection;
    std::vector<std::shared_ptr<LaneWidth>> lane_widths;
};


struct LaneSection : public std::enable_shared_from_this<LaneSection>
{
    LaneSection(double s0, double length);
    void add_lane(std::shared_ptr<Lane> lane);
    void add_lane(std::vector<std::shared_ptr<Lane>> lanes);

    double s0, length;
    std::map<int, std::shared_ptr<Lane>> id2lane;
    std::shared_ptr<Road> road;
    std::vector<std::shared_ptr<Lane>> lanes;
};