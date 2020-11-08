#include "Road.h"

#include <cstring>
#include <iostream>

#include "Lanes.h"

namespace odr
{

Road::Road(double length, int id, int junction)
    : id(id), junction(junction), length(length) {}

void Road::add_lane_section(std::shared_ptr<LaneSection> lane_section)
{
    if (lane_section->road)
        std::cerr << "Error - lane section was already associated with a road" << std::endl;

    lane_section->road = shared_from_this();
    this->lane_sections[lane_section->s0] = lane_section;
}

double Road::get_lane_offset(double s) const
{
    double offs = 0;
    if (this->lane_offsets.size() > 0)
    {
        auto target_lane_offset_iter = this->lane_offsets.upper_bound(s);
        if (target_lane_offset_iter != this->lane_offsets.begin())
            target_lane_offset_iter--;
        offs = target_lane_offset_iter->second->get_offset(s);
    }
    return offs;
}

} // namespace odr