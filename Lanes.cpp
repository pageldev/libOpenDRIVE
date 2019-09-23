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

std::pair<double, double> Lane::get_outer_border_pt(double s)
{    
    int lane_id = this->id;
    double t = 0.0;
    while( lane_id != 0 ) {
        const std::vector<std::shared_ptr<LaneWidth>>& lane_widths_for_lane = this->lanesection->id2lane.at(lane_id)->lane_widths;
        std::shared_ptr<LaneWidth> target_lane_width = lane_widths_for_lane.front();
        for( int idx = 1; idx < lane_widths_for_lane.size(); idx++ ) {
            if( lane_widths_for_lane.at(idx)->s_offset > (s - this->lanesection->s0) ) {
                break;
            } else {
                target_lane_width = lane_widths_for_lane.at(idx);
            }
        }

        double ds = s - this->lanesection->s0 - target_lane_width->s_offset;
        t += target_lane_width->get_width(ds);
        lane_id = (lane_id > 0) ? lane_id-1 : lane_id +1;
    }
    t = (this->id < 0) ? -t : t;
    return lanesection->road->get_refline_point(s, t);
}


LaneSection::LaneSection(double s0, double length)
    : s0(s0), length(length)
{  }

void LaneSection::add_lane(std::shared_ptr<Lane> lane)
{
    lanes.push_back(lane);
    id2lane.insert( std::pair<int, std::shared_ptr<Lane>>( lane->id, lane) );
    lane->lanesection = shared_from_this();
}

void LaneSection::add_lane(std::vector<std::shared_ptr<Lane>> lanes)
{
    for( std::shared_ptr<Lane> lane : lanes ) {
        this->add_lane(lane);
    }
}