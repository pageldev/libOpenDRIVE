#include "Lane.h"

#include <algorithm>
#include <iterator>
#include <type_traits>

namespace odr
{

HeightOffset::HeightOffset(double inner, double outer) : inner(inner), outer(outer) {}

LaneKey::LaneKey(std::string road_id, double lanesection_s0, int lane_id) : road_id(road_id), lanesection_s0(lanesection_s0), lane_id(lane_id) {}

std::string LaneKey::to_string() const { return string_format("%s/%f/%d", this->road_id.c_str(), this->lanesection_s0, this->lane_id); }

Lane::Lane(std::string road_id, double lanesection_s0, int id, bool level, std::string type) :
    key(road_id, lanesection_s0, id), id(id), level(level), type(type)
{
}

std::vector<RoadMark> Lane::get_roadmarks(const double s_start, const double s_end) const
{
    if ((s_start == s_end) || this->roadmark_groups.empty())
        return {};

    auto s_start_rm_iter =
        std::upper_bound(this->roadmark_groups.begin(),
                         this->roadmark_groups.end(),
                         s_start,
                         [](const double& s, const RoadMarkGroup& rmg) -> bool { return s < (rmg.lanesection_s0 + rmg.s_offset); });
    if (s_start_rm_iter != this->roadmark_groups.begin())
        s_start_rm_iter--;

    auto s_end_rm_iter = std::lower_bound(this->roadmark_groups.begin(),
                                          this->roadmark_groups.end(),
                                          s_end,
                                          [](const RoadMarkGroup& rmg, const double& s) -> bool { return (rmg.lanesection_s0 + rmg.s_offset) < s; });

    std::vector<RoadMark> roadmarks;
    for (auto rm_group_iter = s_start_rm_iter; rm_group_iter != s_end_rm_iter; rm_group_iter++)
    {
        const RoadMarkGroup& roadmark_group = *rm_group_iter;

        const double roadmark_group_s0 = roadmark_group.lanesection_s0 + roadmark_group.s_offset;
        const double s_start_roadmark_group = std::max(roadmark_group_s0, s_start);
        const double s_end_roadmark_group = (std::next(rm_group_iter) == s_end_rm_iter)
                                                ? s_end
                                                : std::min(std::next(rm_group_iter)->lanesection_s0 + std::next(rm_group_iter)->s_offset, s_end);

        double width = roadmark_group.weight == "bold" ? ROADMARK_WEIGHT_BOLD_WIDTH : ROADMARK_WEIGHT_STANDARD_WIDTH;
        if (roadmark_group.roadmark_lines.empty())
        {
            if (roadmark_group.width > 0)
                width = roadmark_group.width;

            roadmarks.push_back(RoadMark(this->key.road_id,
                                         this->key.lanesection_s0,
                                         this->id,
                                         roadmark_group_s0,
                                         s_start_roadmark_group,
                                         s_end_roadmark_group,
                                         0,
                                         width,
                                         roadmark_group.type));
        }
        else
        {
            for (const RoadMarksLine& roadmarks_line : roadmark_group.roadmark_lines)
            {
                if (roadmarks_line.width > 0)
                    width = roadmarks_line.width;

                if ((roadmarks_line.length + roadmarks_line.space) == 0)
                    continue;

                const double s0_roadmarks_line = roadmarks_line.group_s0 + roadmarks_line.s_offset;
                for (double s_start_single_roadmark = s0_roadmarks_line; s_start_single_roadmark < s_end_roadmark_group;
                     s_start_single_roadmark += (roadmarks_line.length + roadmarks_line.space))
                {
                    const double s_end_single_roadmark = std::min(s_end, s_start_single_roadmark + roadmarks_line.length);
                    roadmarks.push_back(RoadMark(this->key.road_id,
                                                 this->key.lanesection_s0,
                                                 this->id,
                                                 roadmarks_line.group_s0,
                                                 s_start_single_roadmark,
                                                 s_end_single_roadmark,
                                                 roadmarks_line.t_offset,
                                                 width,
                                                 roadmark_group.type + roadmarks_line.name));
                }
            }
        }
    }

    return roadmarks;
}

} // namespace odr
