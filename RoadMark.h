#pragma once

#include "Math.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace odr
{
struct Lane;
struct RoadMark;

struct RoadMarksLine
{
    RoadMarksLine() = default;
    RoadMarksLine(double width, double length, double space, double tOffset, double sOffset, std::string name, std::string rule);

    double width = -1;
    double length = 0;
    double space = 0;
    double tOffset = 0;
    double sOffset = 0;

    std::string name;
    std::string rule;
};

struct RoadMark
{
    RoadMark() = default;
    RoadMark(double      width,
             double      height,
             double      sOffset,
             std::string type,
             std::string weight,
             std::string color,
             std::string material,
             std::string laneChange);

    double width = -1;
    double height = 0;
    double sOffset = 0;

    std::string type;
    std::string weight;
    std::string color;
    std::string material;
    std::string laneChange;

    std::map<double, RoadMarksLine> s_to_roadmarks_line;
};

struct RoadMarkPolygon
{
    std::vector<Vec3D> outline;
};

} // namespace odr