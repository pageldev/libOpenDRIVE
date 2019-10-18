#pragma once

#include "Road.h"
#include "Geometries.h"

#include <map>
#include <memory>
#include <set>
#include <vector>


class Road;
struct LaneSection;

struct LaneWidth
{
    LaneWidth(double sOffset, double a, double b, double c, double d);
    double get_width(double ds);

    double s_offset, a, b, c, d;
};


struct CmpLaneWidth {
    bool operator()(const std::shared_ptr<LaneWidth>& lhs, const std::shared_ptr<LaneWidth>& rhs) const {
        return (lhs->s_offset < rhs->s_offset); 
    }
};

struct Lane : public std::enable_shared_from_this<Lane>
{
    Lane(int id, std::set<std::shared_ptr<LaneWidth>, CmpLaneWidth> lane_widths);
    Point3D get_outer_border_pt(double s);

    int id;
    std::shared_ptr<LaneSection> lanesection;
    std::set<std::shared_ptr<LaneWidth>, CmpLaneWidth> lane_widths;
};


struct CmpLane {
    bool operator()(const std::shared_ptr<Lane>& lhs, const std::shared_ptr<Lane>& rhs) const {
        return (lhs->id < rhs->id); 
    }
};

struct LaneSection : public std::enable_shared_from_this<LaneSection>
{
    LaneSection(double s0, double length);
    void add_lane(std::shared_ptr<Lane> lane);
    void add_lane(std::vector<std::shared_ptr<Lane>> lanes);

    double s0, length;
    std::shared_ptr<Road> road;
    std::set<std::shared_ptr<Lane>, CmpLane> lanes;
};