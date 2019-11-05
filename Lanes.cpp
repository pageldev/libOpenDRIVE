#include "Lanes.h"

#include <iostream>

LaneOffset::LaneOffset(double s0, double a, double b, double c, double d)
    : s0(s0), a(a), b(b), c(c), d(d)
{  }

double LaneOffset::get_offset(double s)
{
    double ds = s - s0;
    return a + b*ds + c*ds*ds + d*ds*ds*ds;
}

bool CmpLane::operator()(const std::shared_ptr<Lane>& lhs, const std::shared_ptr<Lane>& rhs) const {
    return (lhs->id < rhs->id); 
}


bool CmpLaneWidth::operator()(const std::shared_ptr<LaneWidth>& lhs, const std::shared_ptr<LaneWidth>& rhs) const {
    return (lhs->s_offset < rhs->s_offset); 
}


LaneWidth::LaneWidth(double sOffset, double a, double b, double c, double d)
    : s_offset(sOffset), a(a), b(b), c(c), d(d)
{  }


double LaneWidth::get_width(double ds)
{
    return a + b*ds + c*ds*ds + d*ds*ds*ds;
}


Lane::Lane(int id, std::set<std::shared_ptr<LaneWidth>, CmpLaneWidth> lane_widths)
    : id(id), lane_widths(lane_widths)
{  }


Point3D Lane::get_outer_border_pt(double s)
{    
    int lane_id = this->id;
    double t = 0.0;
 
    std::set<std::shared_ptr<Lane>>::iterator lane_iter = this->lane_section->lanes.find( shared_from_this() );
    while( (*lane_iter)->id != 0 ) {
        std::set<std::shared_ptr<LaneWidth>, CmpLaneWidth>::iterator target_lane_width_iter
            = this->lane_widths.upper_bound(std::make_shared<LaneWidth>(s - this->lane_section->s0, 0.0, 0.0, 0.0, 0.0));
        if( target_lane_width_iter != lane_widths.begin() ) {
            target_lane_width_iter--;
        }
        double ds = s - this->lane_section->s0 - (*target_lane_width_iter)->s_offset;
        t += (*target_lane_width_iter)->get_width(ds);
        lane_iter = ((*lane_iter)->id > 0) ? std::prev(lane_iter) : std::next(lane_iter);
    }

    t = (this->id < 0) ? -t : t;
    std::cout << lane_id << std::endl;
    // if( lane_id == 1) {
    //     for( int i = 0; i < int(t*10); i++ ) std::cout << "#";
    //     std::cout << std::endl;
    // }
    return lane_section->road->get_refline_point(s, t);
}


LaneSection::LaneSection(double s0)
    : s0(s0)
{  }

void LaneSection::add_lane(std::shared_ptr<Lane> lane) 
{
    if( lane->lane_section ) {
        std::cerr << "Error - lane was already associated with a lane section" << std::endl;
    }
    lane->lane_section = shared_from_this();
    this->lanes.insert(lane);
}