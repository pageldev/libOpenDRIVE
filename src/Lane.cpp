#include "Lane.h"
#include "Log.hpp"
#include "RoadMark.h"

#include <algorithm>
#include <fmt/format.h>
#include <iterator>

namespace odr
{

HeightOffset::HeightOffset(double inner, double outer) : inner(inner), outer(outer) {}

LaneKey::LaneKey(std::string road_id, double lanesection_s0, int lane_id) : road_id(road_id), lanesection_s0(lanesection_s0), lane_id(lane_id) {}

std::string LaneKey::to_string() const
{
    return fmt::format("{}/{:.17g}/{}", this->road_id, this->lanesection_s0, this->lane_id);
}

Lane::Lane(std::string road_id, double lanesection_s0, int id, bool level, std::string type) :
    key(road_id, lanesection_s0, id), id(id), level(level), type(type)
{
}

std::vector<SingleRoadMark> Lane::get_roadmarks(const double s_start, const double s_end) const
{
    if ((s_start == s_end) || this->s_to_roadmark_group.empty())
        return {};

    // OpenDRIVE Format Specification, Rev. 1.8.1, 11.8 Road markings:
    // "The <roadMark> elements of a lane shall remain valid until another <roadMark> element starts or the lane section ends."
    auto rm_iter_start = this->s_to_roadmark_group.upper_bound(s_start); // first element > s
    if (rm_iter_start != this->s_to_roadmark_group.begin())
        rm_iter_start--;
    auto rm_iter_end = this->s_to_roadmark_group.lower_bound(s_end); // first element >= s

    std::vector<SingleRoadMark> roadmarks;
    for (auto rm_group_iter = rm_iter_start; rm_group_iter != rm_iter_end; rm_group_iter++)
    {
        const double         roadmark_group_s0 = rm_group_iter->first;
        const RoadMarkGroup& roadmark_group = rm_group_iter->second;

        const double s_start_roadmark_group = std::max(roadmark_group_s0, s_start);
        const double s_end_roadmark_group = (std::next(rm_group_iter) == rm_iter_end) ? s_end : std::min(std::next(rm_group_iter)->first, s_end);

        double width = RoadMarkGroup::StandardWidth;
        if (roadmark_group.width)
            width = roadmark_group.width.value();
        else if (roadmark_group.weight.value_or("standard") == "bold")
            width = RoadMarkGroup::BoldWidth;

        if (roadmark_group.type_elem)
        {
            for (const RoadMarkLine& roadmark_line : roadmark_group.type_elem->lines)
            {
                if (roadmark_line.length < 1e-9)
                    continue;

                width = roadmark_line.width.value_or(width);
                const double space = roadmark_line.space.value_or(0);

                const double s0_roadmarks_line = roadmark_group_s0 + roadmark_line.sOffset;
                for (double s_start_single_roadmark = s0_roadmarks_line; s_start_single_roadmark < s_end_roadmark_group;
                     s_start_single_roadmark += (roadmark_line.length + space))
                {
                    const double s_end_single_roadmark = std::min(s_end, s_start_single_roadmark + roadmark_line.length);
                    roadmarks.emplace_back(s_start_single_roadmark, s_end_single_roadmark, roadmark_line.tOffset, width, roadmark_group.type);
                    if (space < 1e-9) // treat roadmark <line> with space = 0 as single roadmark
                        break;
                }
            }
        }
        else
        {
            roadmarks.emplace_back(s_start_roadmark_group, s_end_roadmark_group, 0, width, roadmark_group.type);
        }
    }

    return roadmarks;
}

} // namespace odr
