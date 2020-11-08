#include "Lanes.h"

#include <iostream>

namespace odr
{

LaneOffset::LaneOffset(double s0, double a, double b, double c, double d)
    : s0(s0), a(a), b(b), c(c), d(d)
{
}

double LaneOffset::get_offset(double s) const
{
    const double ds = s - s0;
    return a + b * ds + c * ds * ds + d * ds * ds * ds;
}

LaneWidth::LaneWidth(double sOffset, double a, double b, double c, double d)
    : s_offset(sOffset), a(a), b(b), c(c), d(d)
{
}

double LaneWidth::get_width(double ds) const
{
    return a + b * ds + c * ds * ds + d * ds * ds * ds;
}

Lane::Lane(int id, std::string type, std::map<double, std::shared_ptr<LaneWidth>> lane_widths)
    : id(id), type(type), lane_widths(lane_widths)
{
}

Vec3D Lane::get_outer_border_pt(double s) const
{
    double t = 0.0;

    auto lane_iter = this->lane_section->lanes.find(this->id);
    while (lane_iter->second->id != 0)
    {
        auto target_lane_width_iter = lane_iter->second->lane_widths.upper_bound(s - this->lane_section->s0);
        if (target_lane_width_iter != lane_widths.begin())
            target_lane_width_iter--;
        const double ds = s - this->lane_section->s0 - target_lane_width_iter->second->s_offset;
        t += target_lane_width_iter->second->get_width(ds);
        lane_iter = (lane_iter->second->id > 0) ? std::prev(lane_iter) : std::next(lane_iter);
    }

    t = (this->id < 0) ? -t : t;

    double t_offset = 0;
    if (this->lane_section->road->lane_offsets.size() > 0)
    {
        auto target_lane_offset_iter = this->lane_section->road->lane_offsets.upper_bound(s);
        if (target_lane_offset_iter != this->lane_section->road->lane_offsets.begin())
            target_lane_offset_iter--;
        t_offset = target_lane_offset_iter->second->get_offset(s);
    }

    return this->lane_section->road->ref_line->get_point(s, t, t_offset);
}

LaneSection::LaneSection(double s0)
    : s0(s0)
{
}

void LaneSection::add_lane(std::shared_ptr<Lane> lane)
{
    if (lane->lane_section)
    {
        std::cerr << "Error - lane was already associated with a lane section" << std::endl;
    }
    lane->lane_section = shared_from_this();
    this->lanes[lane->id] = lane;
}

} // namespace odr