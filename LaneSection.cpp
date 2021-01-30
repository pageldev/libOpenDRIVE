#include "LaneSection.h"
#include "RefLine.h"
#include "Road.h"

#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

namespace odr
{
LaneSection::LaneSection(double s0) : s0(s0) {}

double LaneSection::get_end() const
{
    if (auto road_ptr = this->road.lock())
    {
        auto s_lanesec_iter = road_ptr->s_to_lanesection.find(this->s0);
        if (s_lanesec_iter == road_ptr->s_to_lanesection.end())
            throw std::runtime_error("road associated with wrong lane section");

        const bool   is_last = (s_lanesec_iter == std::prev(road_ptr->s_to_lanesection.end()));
        const double next_s0 = is_last ? road_ptr->length : std::next(s_lanesec_iter)->first;

        return next_s0 - std::numeric_limits<double>::min();
    }
    else
    {
        throw std::runtime_error("could not access parent road for lane section");
    }

    return 0;
}

ConstLaneSet LaneSection::get_lanes() const
{
    ConstLaneSet lanes;
    for (const auto& id_lane : this->id_to_lane)
        lanes.insert(id_lane.second);

    return lanes;
}

LaneSet LaneSection::get_lanes()
{
    LaneSet lanes;
    for (const auto& id_lane : this->id_to_lane)
        lanes.insert(id_lane.second);

    return lanes;
}

std::shared_ptr<const Lane> LaneSection::get_lane(double s, double t) const
{
    if (this->id_to_lane.at(0)->outer_border.get(s) == t) // exactly on lane #0
        return id_to_lane.at(0);

    std::map<double, int> outer_border_to_lane_id;
    for (const auto& id_lane : id_to_lane)
        outer_border_to_lane_id[id_lane.second->outer_border.get(s)] = id_lane.first;

    auto target_iter = outer_border_to_lane_id.lower_bound(t);
    if (target_iter == outer_border_to_lane_id.end()) // past upper boundary
        target_iter--;

    if (target_iter->second <= 0 && target_iter != outer_border_to_lane_id.begin() && t != target_iter->first)
        target_iter--;

    return this->id_to_lane.at(target_iter->second);
}

std::shared_ptr<Lane> LaneSection::get_lane(double s, double t)
{
    std::shared_ptr<Lane> lane = std::const_pointer_cast<Lane>(static_cast<const LaneSection&>(*this).get_lane(s, t));
    return lane;
}

// std::vector<RoadMarkLines> LaneSection::get_roadmark_lines(int lane_id, double resolution) const
// {
//     if (auto road_ptr = this->road.lock())
//     {
//         std::vector<RoadMarkLines> out_roadmarks;

//         const double s_end = this->get_end();
//         const auto&  lane = this->id_to_lane.at(lane_id);

//         for (auto s_roadmarks_iter = lane->s_to_roadmark.begin(); s_roadmarks_iter != lane->s_to_roadmark.end(); s_roadmarks_iter++)
//         {
//             const RoadMark& roadmark = s_roadmarks_iter->second;

//             const bool is_last = (s_roadmarks_iter == std::prev(lane->s_to_roadmark.end()));
//             double     s_end_roadmark = is_last ? s_end : std::min(s_end, std::next(s_roadmarks_iter)->first - std::numeric_limits<double>::min());
//             double     width = roadmark.weight == "bold" ? ROADMARK_WEIGHT_BOLD_WIDTH : ROADMARK_WEIGHT_STANDARD_WIDTH;

//             if (roadmark.s_to_roadmarks_line.size() == 0)
//             {
//                 std::vector<double> s_vals;
//                 for (double s = s_roadmarks_iter->first; s < s_end_roadmark; s += resolution)
//                     s_vals.push_back(s);
//                 s_vals.push_back(s_end_roadmark);

//                 if (roadmark.width > 0)
//                     width = roadmark.width;

//                 RoadMarkLines roadmark_lines{lane->id,
//                                              width,
//                                              0,
//                                              0,
//                                              roadmark.height,
//                                              "",
//                                              "",
//                                              roadmark.type,
//                                              roadmark.weight,
//                                              roadmark.color,
//                                              roadmark.material,
//                                              roadmark.laneChange};

//                 Line3D line;
//                 for (const double& s : s_vals)
//                     line.push_back(lane->get_surface_pt(s, lane->outer_border.get(s)));
//                 roadmark_lines.lines.push_back(std::move(line));

//                 out_roadmarks.push_back(std::move(roadmark_lines));
//             }
//             else
//             {
//                 /* multiple parallel roadmarks lines possible */
//                 for (const auto& s_roadmarks_line : roadmark.s_to_roadmarks_line)
//                 {
//                     const RoadMark::RoadMarksLine& roadmarks_line = s_roadmarks_line.second;
//                     if (roadmarks_line.width > 0)
//                         width = roadmarks_line.width;

//                     RoadMarkLines roadmark_lines{lane->id,
//                                                  width,
//                                                  roadmarks_line.length,
//                                                  roadmarks_line.space,
//                                                  roadmark.height,
//                                                  roadmarks_line.name,
//                                                  roadmarks_line.rule,
//                                                  roadmark.type,
//                                                  roadmark.weight,
//                                                  roadmark.color,
//                                                  roadmark.material,
//                                                  roadmark.laneChange};

//                     for (double s_start_single_roadmark = s_roadmarks_line.first; s_start_single_roadmark < s_end_roadmark;
//                          s_start_single_roadmark += (roadmarks_line.length + roadmarks_line.space))
//                     {
//                         const double s_end_single_roadmark = std::min(s_end, s_start_single_roadmark + roadmarks_line.length);

//                         std::vector<double> s_vals;
//                         for (double s = s_start_single_roadmark; s < s_end_single_roadmark; s += resolution)
//                             s_vals.push_back(s);
//                         s_vals.push_back(s_end_single_roadmark);

//                         Line3D line;
//                         for (const double& s : s_vals)
//                             line.push_back(lane->get_surface_pt(s, lane->outer_border.get(s) + roadmarks_line.tOffset));
//                         roadmark_lines.lines.push_back(std::move(line));
//                     }

//                     out_roadmarks.push_back(std::move(roadmark_lines));
//                 }
//             }
//         }

//         return out_roadmarks;
//     }
//     else
//     {
//         throw std::runtime_error("could not access parent road for lane section");
//     }

//     return {};
// }

// std::vector<RoadMarkLines> LaneSection::get_roadmark_lines(double resolution) const
// {
//     std::vector<RoadMarkLines> roadmark_lines;
//     for (const auto& id_lane : this->id_to_lane)
//     {
//         std::vector<RoadMarkLines> lane_roadmarks = this->get_roadmark_lines(id_lane.first, resolution);
//         roadmark_lines.insert(roadmark_lines.end(), lane_roadmarks.begin(), lane_roadmarks.end());
//     }

//     return roadmark_lines;
// }

} // namespace odr