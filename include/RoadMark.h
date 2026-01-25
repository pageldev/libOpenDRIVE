#pragma once
#include "Utils.hpp"

#include <functional>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace odr
{

struct RoadMarkLine
{
    RoadMarkLine(double                     sOffset,
                 double                     tOffset,
                 double                     length,
                 std::optional<double>      width = std::nullopt,
                 std::optional<double>      space = std::nullopt,
                 std::optional<std::string> color = std::nullopt,
                 std::optional<std::string> rule = std::nullopt);

    double sOffset;
    double tOffset;
    double length;

    std::optional<double> width;
    std::optional<double> space;

    std::optional<std::string> color;
    std::optional<std::string> rule;
};

struct RoadMarkType
{
    RoadMarkType(std::string name, std::optional<double> width = std::nullopt);

    std::string           name;
    std::optional<double> width; // common that xodrs don't set this

    std::vector<RoadMarkLine> lines;
};

struct RoadMark
{
    RoadMark(double                     s_offset,
             std::string                type,
             std::string                color,
             std::optional<double>      width = std::nullopt,
             std::optional<double>      height = std::nullopt,
             std::optional<std::string> weight = std::nullopt,
             std::optional<std::string> material = std::nullopt,
             std::optional<std::string> lane_change = std::nullopt);

    static constexpr double StandardWidth = 0.12;
    static constexpr double BoldWidth = 0.25;

    std::string road_id;
    double      lanesection_s0;
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
    SingleRoadMark(double s0, double s1, double t, double width, std::string type) noexcept;

    double s0;
    double s1;
    double t;
    double width;

    std::string type;
};

} // namespace odr
