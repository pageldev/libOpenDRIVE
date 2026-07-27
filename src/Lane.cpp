#include "libodr/Lane.h"
#include "libodr/RoadMark.h"
#include "libodr/Utils.hpp"

#include <algorithm>
#include <fmt/format.h>
#include <iterator>

namespace odr
{

HeightOffset::HeightOffset(double s_offset, double inner, double outer) : s_offset(s_offset), inner(inner), outer(outer)
{
    require_or_throw(s_offset >= 0, "s must be greater than or equal to 0 (got {})", s_offset);
    require_or_throw(!std::isnan(inner), "inner border must not be NaN");
    require_or_throw(!std::isnan(outer), "outer border must not be NaN");
}

LaneKey::LaneKey(const std::string& road_id, double lane_section_s, int lane_id) : road_id(road_id), lane_section_s(lane_section_s), lane_id(lane_id)
{
}

std::string LaneKey::to_string() const
{
    return fmt::format("{}/{:.17g}/{}", this->road_id, this->lane_section_s, this->lane_id);
}

Lane::Lane(int id, std::optional<std::string> type, std::optional<bool> level) : id(id), type(type), level(level) {}

std::vector<SingleRoadMark> Lane::get_roadmarks(double s_start, double s_end) const
{
    if ((s_start == s_end) || this->s_to_road_mark.empty())
        return {};

    // OpenDRIVE Format Specification, Rev. 1.8.1, 11.8 Road markings:
    // "The <roadMark> elements of a lane shall remain valid until another <roadMark> element starts or the lane section ends."
    auto rm_iter_start = this->s_to_road_mark.upper_bound(s_start); // first element > s
    if (rm_iter_start != this->s_to_road_mark.begin())
        rm_iter_start--;
    auto rm_iter_end = this->s_to_road_mark.lower_bound(s_end); // first element >= s

    std::vector<SingleRoadMark> roadmarks;
    for (auto rm_iter = rm_iter_start; rm_iter != rm_iter_end; rm_iter++)
    {
        const double    s_road_mark = rm_iter->first;
        const RoadMark& roadmark = rm_iter->second;

        double width = RoadMark::StandardWidth;
        if (roadmark.width)
            width = roadmark.width.value();
        else if (roadmark.weight.value_or("standard") == "bold")
            width = RoadMark::BoldWidth;

        const double s_end_road_mark = (std::next(rm_iter) == rm_iter_end) ? s_end : std::min(std::next(rm_iter)->first, s_end);

        if (roadmark.type_elem)
        {
            for (const RoadMarkLine& rm_line : roadmark.type_elem->lines)
            {
                if (is_zero(rm_line.length))
                    continue;

                width = rm_line.width.value_or(width);
                const double space = rm_line.space.value_or(0);

                const double s_road_mark_line = s_road_mark + rm_line.s_offset;
                for (double s_single_road_mark = s_road_mark_line; s_single_road_mark < s_end_road_mark;
                     s_single_road_mark += (rm_line.length + space))
                {
                    const double s_end_single_road_mark = std::min(s_end, s_single_road_mark + rm_line.length);
                    roadmarks.emplace_back(s_single_road_mark, s_end_single_road_mark, rm_line.t_offset, width, roadmark.type);
                    if (is_zero(space)) // treat roadMark::type::line with space = 0 as single roadmark
                        break;
                }
            }
        }
        else
        {
            roadmarks.emplace_back(std::max(s_road_mark, s_start), s_end_road_mark, 0, width, roadmark.type);
        }
    }

    return roadmarks;
}

} // namespace odr
