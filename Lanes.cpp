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


LaneWidth::LaneWidth(double sOffset, double a, double b, double c, double d)
    : s_offset(sOffset), a(a), b(b), c(c), d(d)
{  }


double LaneWidth::get_width(double ds)
{
    return a + b*ds + c*ds*ds + d*ds*ds*ds;
}


Lane::Lane(int id, std::string type, std::map<double, std::shared_ptr<LaneWidth>> lane_widths)
    : id(id), type(type), lane_widths(lane_widths)
{  }


Point3D Lane::get_outer_border_pt(double s)
{    
    int lane_id = this->id;
    double t = 0.0;
 
    std::map<int, std::shared_ptr<Lane>>::iterator lane_iter = this->lane_section->lanes.find( this->id );
    while( (*lane_iter).second->id != 0 ) {
        std::map<double, std::shared_ptr<LaneWidth>>::iterator target_lane_width_iter = (*lane_iter).second->lane_widths.upper_bound(s - this->lane_section->s0);
        if( target_lane_width_iter != lane_widths.begin() ) {
            target_lane_width_iter--;
        }
        double ds = s - this->lane_section->s0 - (*target_lane_width_iter).second->s_offset;
        t += (*target_lane_width_iter).second->get_width(ds);
        lane_iter = ((*lane_iter).second->id > 0) ? std::prev(lane_iter) : std::next(lane_iter);
    }

    t = (this->id < 0) ? -t : t;
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
    this->lanes[lane->id] = lane;
}