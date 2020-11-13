#include "Lanes.h"

namespace odr
{

Lane::Lane(int id, std::string type)
    : id(id), type(type)
{
}

Vec3D Lane::get_outer_border_pt(double s) const
{
    double t = 0.0;

    auto lane_iter = this->lane_section->lanes.find(this->id);
    while (lane_iter->second->id != 0)
    {
        t += lane_iter->second->lane_width.get(s - this->lane_section->s0);
        lane_iter = (lane_iter->second->id > 0) ? std::prev(lane_iter) : std::next(lane_iter);
    }

    t = (this->id < 0) ? -t : t;

    const double t_offset = this->lane_section->road->lane_offset.get(s);
    return this->lane_section->road->get_xyz(s, t + t_offset, 0.0);
}

LaneSection::LaneSection(double s0)
    : s0(s0)
{
}

} // namespace odr