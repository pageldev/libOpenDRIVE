#pragma once

#include "Road.h"

#include <memory>
#include <set>


struct Lane;

struct LaneWidth
{
    LaneWidth(double sOffset, double a, double b, double c, double d);
    double get_width(double ds);

    double s_offset, a, b, c, d;
};


struct CmpLaneWidth 
{
    bool operator()(const std::shared_ptr<LaneWidth>& lhs, const std::shared_ptr<LaneWidth>& rhs) const;
};


struct CmpLane 
{
    bool operator()(const std::shared_ptr<Lane>& lhs, const std::shared_ptr<Lane>& rhs) const;
};

struct LaneSection
{
    static std::shared_ptr<LaneSection> create_lane_section(double s0, std::shared_ptr<Road> road) 
    {
        std::shared_ptr<LaneSection> lane_section(new LaneSection(s0, road));
        road->lane_sections.insert(lane_section);
        return lane_section;
    }
    double s0;
    std::shared_ptr<Road> road;
    std::set<std::shared_ptr<Lane>, CmpLane> lanes;

    private:
        LaneSection(double s0, std::shared_ptr<Road> road);
};


/* Lane factory method uses Lanesection - therefore this order, fw-declaring Lane instead of LaneSection */
struct Lane : public std::enable_shared_from_this<Lane>
{
    /* Factory method to register this with parent lane_section (shared_from_this in CTOR not possible) */
    static std::shared_ptr<Lane> create_lane(int id, std::shared_ptr<LaneSection> lane_section, std::set<std::shared_ptr<LaneWidth>, CmpLaneWidth> lane_widths) 
    {
        std::shared_ptr<Lane> lane(new Lane(id, lane_section, lane_widths));
        lane_section->lanes.insert(lane);
        return lane;
    }
    Point3D get_outer_border_pt(double s);

    int id;
    std::shared_ptr<LaneSection> lane_section;
    std::set<std::shared_ptr<LaneWidth>, CmpLaneWidth> lane_widths;

    private:
        Lane(int id, std::shared_ptr<LaneSection> lane_section, std::set<std::shared_ptr<LaneWidth>, CmpLaneWidth> lane_widths);
};