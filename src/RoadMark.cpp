#include "RoadMark.h"

namespace odr
{
RoadMarksLine::RoadMarksLine(std::string road_id,
                             double      lanesection_s0,
                             int         lane_id,
                             double      group_s0,
                             double      width,
                             double      length,
                             double      space,
                             double      t_offset,
                             double      s_offset,
                             std::string name,
                             std::string rule) :
    road_id(road_id),
    lanesection_s0(lanesection_s0), lane_id(lane_id), group_s0(group_s0), width(width), length(length), space(space), t_offset(t_offset),
    s_offset(s_offset), name(name), rule(rule)
{
}

RoadMarkGroup::RoadMarkGroup(std::string road_id,
                             double      lanesection_s0,
                             int         lane_id,
                             double      width,
                             double      height,
                             double      s_offset,
                             std::string type,
                             std::string weight,
                             std::string color,
                             std::string material,
                             std::string lane_change) :
    road_id(road_id),
    lanesection_s0(lanesection_s0), lane_id(lane_id), width(width), height(height), s_offset(s_offset), type(type), weight(weight), color(color),
    material(material), lane_change(lane_change)
{
}

RoadMark::RoadMark(std::string road_id,
                   double      lanesection_s0,
                   int         lane_id,
                   double      group_s0,
                   double      s_start,
                   double      s_end,
                   double      t_offset,
                   double      width,
                   std::string type) :
    road_id(road_id),
    lanesection_s0(lanesection_s0), lane_id(lane_id), group_s0(group_s0), s_start(s_start), s_end(s_end), t_offset(t_offset), width(width), type(type)
{
}

} // namespace odr