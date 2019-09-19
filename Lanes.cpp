#include "Lanes.h"

LaneWidth::LaneWidth(double sOffset, double a, double b, double c, double d)
    : s_offset(sOffset), a(a), b(b), c(c), d(d)
{  }

double LaneWidth::get_width(double ds)
{
    return a + b*ds + c*ds*ds + d*ds*ds*ds;
}


Lane::Lane(int id, std::vector<std::shared_ptr<LaneWidth>> lane_widths)
    : id(id), lane_widths(lane_widths)
{
    std::sort(lane_widths.begin(), lane_widths.end()
        , [](std::shared_ptr<LaneWidth> a, std::shared_ptr<LaneWidth> b){ return a->s_offset < b->s_offset; } );
}

std::pair<double, double> Lane::get_lane_border_pt(double s)
{    
    std::shared_ptr<LaneWidth> target_lane_width = lane_widths.front();
    for( int idx = 1; idx < lane_widths.size(); idx++ ) {
        if( lane_widths.at(idx)->s_offset > s ) {
            target_lane_width = lane_widths.at(idx - 1);
            break;
        }
    }
    double ds = s - lanesection->s0 - target_lane_width->s_offset;
    double t = target_lane_width->get_width(ds);
    t = (id < 0) ? -t : t;
    return lanesection->road->get_refline_point(s, t);
}


LaneSection::LaneSection(double s0, double length)
    : s0(s0), length(length)
{  }

void LaneSection::add_lane(std::shared_ptr<Lane> lane)
{
    lanes.push_back(lane);
    lane->lanesection = shared_from_this();
}

void LaneSection::add_lane(std::vector<std::shared_ptr<Lane>> lanes)
{
    for( std::shared_ptr<Lane> lane : lanes ) {
        this->add_lane(lane);
    }
}