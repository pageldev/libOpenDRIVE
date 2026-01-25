#include "Lane.h"
#include "Log.hpp"
#include "RoadMark.h"
#include "Utils.hpp"

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
    if ((s_start == s_end) || this->s_to_roadmark.empty())
        return {};

    // OpenDRIVE Format Specification, Rev. 1.8.1, 11.8 Road markings:
    // "The <roadMark> elements of a lane shall remain valid until another <roadMark> element starts or the lane section ends."
    auto rm_iter_start = this->s_to_roadmark.upper_bound(s_start); // first element > s
    if (rm_iter_start != this->s_to_roadmark.begin())
        rm_iter_start--;
    auto rm_iter_end = this->s_to_roadmark.lower_bound(s_end); // first element >= s

    std::vector<SingleRoadMark> roadmarks;
    for (auto rm_iter = rm_iter_start; rm_iter != rm_iter_end; rm_iter++)
    {
        const double    roadmark_s0 = rm_iter->first;
        const RoadMark& roadmark = rm_iter->second;

        double width = RoadMark::StandardWidth;
        if (roadmark.width)
            width = roadmark.width.value();
        else if (roadmark.weight.value_or("standard") == "bold")
            width = RoadMark::BoldWidth;

        const double s_end_roadmark = (std::next(rm_iter) == rm_iter_end) ? s_end : std::min(std::next(rm_iter)->first, s_end);

        if (roadmark.type_elem)
        {
            for (const RoadMarkLine& rm_line : roadmark.type_elem->lines)
            {
                if (is_zero(rm_line.length))
                    continue;

                width = rm_line.width.value_or(width);
                const double space = rm_line.space.value_or(0);

                const double s0_roadmarks_line = roadmark_s0 + rm_line.sOffset;
                for (double s_single_rm = s0_roadmarks_line; s_single_rm < s_end_roadmark; s_single_rm += (rm_line.length + space))
                {
                    const double s_end_single_rm = std::min(s_end, s_single_rm + rm_line.length);
                    roadmarks.emplace_back(s_single_rm, s_end_single_rm, rm_line.tOffset, width, roadmark.type);
                    if (is_zero(space)) // treat roadMark::type::line with space = 0 as single roadmark
                        break;
                }
            }
        }
        else
        {
            roadmarks.emplace_back(std::max(roadmark_s0, s_start), s_end_roadmark, 0, width, roadmark.type);
        }
    }

    return roadmarks;
}

} // namespace odr
