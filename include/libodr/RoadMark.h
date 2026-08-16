#pragma once
#include "libodr/Utils.hpp"

#include <functional>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace odr
{

struct RoadMarkLine
{
    RoadMarkLine(double                     s_offset,
                 double                     t_offset,
                 double                     length,
                 std::optional<double>      width = std::nullopt,
                 std::optional<double>      space = std::nullopt,
                 std::optional<std::string> color = std::nullopt,
                 std::optional<std::string> rule = std::nullopt);

    double s_offset;
    double t_offset;
    double length;

    std::optional<double> width;
    std::optional<double> space;

    std::optional<std::string> color;
    std::optional<std::string> rule;
};

struct RoadMarkType
{
    RoadMarkType(const std::string& name, std::optional<double> width = std::nullopt);

    std::string           name;
    std::optional<double> width; // can be superseded by RoadMarkLine width (Rev. 1.4, 5.3.7.2.1.1.4.1.1)

    std::vector<RoadMarkLine> lines;
};

struct RoadMark
{
    RoadMark(double                     s_offset,
             const std::string&         type,
             const std::string&         color,
             std::optional<double>      width = std::nullopt,
             std::optional<double>      height = std::nullopt,
             std::optional<std::string> weight = std::nullopt,
             std::optional<std::string> material = std::nullopt,
             std::optional<std::string> lane_change = std::nullopt);

    static constexpr double StandardWidth = 0.12;
    static constexpr double BoldWidth = 0.25;

    std::string road_id;
    double      lane_section_s;
    int         lane_id;
    double      s_offset;

    std::string type;
    std::string color;

    std::optional<double> width;
    std::optional<double> height;

    std::optional<std::string> weight;
    std::optional<std::string> material;
    std::optional<std::string> lane_change;

    std::optional<RoadMarkType> type_elem;
};

struct SingleRoadMark
{
    SingleRoadMark(double s_start, double s_end, double t_offset, double width, const std::string& type) noexcept;

    double s_start;
    double s_end;
    double t_offset;
    double width;

    std::string type;
};

} // namespace odr
