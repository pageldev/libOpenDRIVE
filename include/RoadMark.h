#pragma once
#include "Utils.hpp"
#include "XmlNode.h"

#include <functional>
#include <set>
#include <string>

namespace odr
{
const double ROADMARK_WEIGHT_STANDARD_WIDTH = 0.12;
const double ROADMARK_WEIGHT_BOLD_WIDTH = 0.25;

struct RoadMarksLine : public XmlNode
{
    RoadMarksLine(std::string road_id,
                  double      lanesection_s0,
                  int         lane_id,
                  double      group_s0,
                  double      width,
                  double      length,
                  double      space,
                  double      t_offset,
                  double      s_offset,
                  std::string name,
                  std::string rule);

    std::string road_id = "";
    double      lanesection_s0 = 0;
    int         lane_id = 0;
    double      group_s0 = 0;

    double width = -1;
    double length = 0;
    double space = 0;
    double t_offset = 0;
    double s_offset = 0;

    std::string name = "";
    std::string rule = "";
};

} // namespace odr

namespace std
{
template<>
struct less<odr::RoadMarksLine>
{
    bool operator()(const odr::RoadMarksLine& lhs, const odr::RoadMarksLine& rhs) const
    {
        return odr::compare_class_members(lhs,
                                          rhs,
                                          less<void>(),
                                          &odr::RoadMarksLine::road_id,
                                          &odr::RoadMarksLine::lanesection_s0,
                                          &odr::RoadMarksLine::lane_id,
                                          &odr::RoadMarksLine::group_s0,
                                          &odr::RoadMarksLine::width,
                                          &odr::RoadMarksLine::length,
                                          &odr::RoadMarksLine::space,
                                          &odr::RoadMarksLine::t_offset,
                                          &odr::RoadMarksLine::s_offset,
                                          &odr::RoadMarksLine::name,
                                          &odr::RoadMarksLine::rule);
    }
};
} // namespace std

namespace odr
{

struct RoadMarkGroup : public XmlNode
{
    RoadMarkGroup(std::string road_id,
                  double      lanesection_s0,
                  int         lane_id,
                  double      width,
                  double      height,
                  double      s_offset,
                  std::string type,
                  std::string weight,
                  std::string color,
                  std::string material,
                  std::string lane_change);

    std::string road_id = "";
    double      lanesection_s0 = 0;
    int         lane_id = 0;

    double width = -1;
    double height = 0;
    double s_offset = 0;

    std::string type = "";
    std::string weight = "";
    std::string color = "";
    std::string material = "";
    std::string lane_change = "";

    std::set<RoadMarksLine> roadmark_lines;
};

struct RoadMark
{
    RoadMark(std::string road_id,
             double      lanesection_s0,
             int         lane_id,
             double      group_s0,
             double      s_start,
             double      s_end,
             double      t_offset,
             double      width,
             std::string type);

    std::string road_id = "";
    double      lanesection_s0 = 0;
    int         lane_id = 0;
    double      group_s0 = 0;

    double s_start = 0;
    double s_end = 0;
    double t_offset = 0;
    double width = 0;

    std::string type = "";
};

} // namespace odr

namespace std
{
template<>
struct less<odr::RoadMarkGroup>
{
    bool operator()(const odr::RoadMarkGroup& lhs, const odr::RoadMarkGroup& rhs) const
    {
        return odr::compare_class_members(lhs,
                                          rhs,
                                          less<void>(),
                                          &odr::RoadMarkGroup::road_id,
                                          &odr::RoadMarkGroup::lanesection_s0,
                                          &odr::RoadMarkGroup::lane_id,
                                          &odr::RoadMarkGroup::s_offset,
                                          &odr::RoadMarkGroup::width,
                                          &odr::RoadMarkGroup::height,
                                          &odr::RoadMarkGroup::type,
                                          &odr::RoadMarkGroup::weight,
                                          &odr::RoadMarkGroup::color,
                                          &odr::RoadMarkGroup::material,
                                          &odr::RoadMarkGroup::lane_change);
    }
};
} // namespace std