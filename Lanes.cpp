#include "Lanes.h"
#include "Road.h"

#include <iterator>
#include <stdexcept>
#include <utility>

namespace odr
{
Lane::Lane(int id, std::string type) : id(id), type(type) {}

Vec3D Lane::get_outer_border_pt(double s) const
{
    double t = 0.0;

    auto lane_iter = this->lane_section->id_to_lane.find(this->id);
    while (lane_iter->second->id != 0)
    {
        t += lane_iter->second->lane_width.get(s - this->lane_section->s0);
        lane_iter = (lane_iter->second->id > 0) ? std::prev(lane_iter) : std::next(lane_iter);
    }

    t = (this->id < 0) ? -t : t;

    const double t_offset = this->lane_section->road->lane_offset.get(s);
    return this->lane_section->road->get_xyz(s, t + t_offset, 0.0);
}

LaneSection::LaneSection(double s0) : s0(s0) {}

LaneSet LaneSection::get_lanes()
{
    LaneSet lanes;
    for (const auto& id_lane : this->id_to_lane)
        lanes.insert(id_lane.second);

    return lanes;
}

std::map<int, double> LaneSection::get_lane_borders(double s) const
{
    auto id_lane_iter0 = this->id_to_lane.find(0);
    if (id_lane_iter0 == this->id_to_lane.end())
        throw std::runtime_error("lane section does not have lane #0");

    std::map<int, double> id_to_outer_border;

    /* iterate from id #0 towards +inf */
    auto id_lane_iter1 = std::next(id_lane_iter0);
    for (auto iter = id_lane_iter1; iter != this->id_to_lane.end(); iter++)
    {
        const double lane_width = iter->second->lane_width.get(s);
        id_to_outer_border[iter->first] = (iter == id_lane_iter1) ? lane_width : lane_width + id_to_outer_border.at(std::prev(iter)->first);
    }

    /* iterate from id #0 towards -inf */
    std::map<int, std::shared_ptr<Lane>>::const_reverse_iterator r_id_lane_iter_1(id_lane_iter0);
    for (auto r_iter = r_id_lane_iter_1; r_iter != id_to_lane.rend(); r_iter++)
    {
        const double lane_width = r_iter->second->lane_width.get(s);
        id_to_outer_border[r_iter->first] = (r_iter == r_id_lane_iter_1) ? lane_width : lane_width + id_to_outer_border.at(std::prev(r_iter)->first);
    }

    return id_to_outer_border;
}

} // namespace odr