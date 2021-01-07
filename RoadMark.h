#pragma once

#include "Math.hpp"

#include <memory>
#include <string>
#include <vector>

namespace odr
{
struct Lane;
struct RoadMark;

struct RoadMarkLine
{
    RoadMarkLine() = default;
    RoadMarkLine(double width, double length, double space, double tOffset, double sOffset, std::string name, std::string rule);

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
    RoadMark(double width, double height, std::string type, std::string weight, std::string color, std::string material, std::string laneChange);

    double width = -1;
    double height = 0;

    std::string type;
    std::string weight;
    std::string color;
    std::string material;
    std::string laneChange;

    std::vector<RoadMarkLine> lines;
};

} // namespace odr