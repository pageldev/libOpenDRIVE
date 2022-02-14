#pragma once

#include "Math.hpp"
#include "Utils.hpp"
#include "XmlNode.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace odr
{
const double ROADMARK_WEIGHT_STANDARD_WIDTH = 0.12;
const double ROADMARK_WEIGHT_BOLD_WIDTH = 0.25;

struct RoadMarksLine : public XmlNode
{
    RoadMarksLine() = default;

    double width = -1;
    double length = 0;
    double space = 0;
    double t_offset = 0;
    double s_offset = 0;

    std::string name;
    std::string rule;
};

struct RoadMarkGroup : public XmlNode
{
    RoadMarkGroup() = default;

    double width = -1;
    double height = 0;
    double s_offset = 0;

    std::string type;
    std::string weight;
    std::string color;
    std::string material;
    std::string laneChange;

    std::map<double, RoadMarksLine> s_to_roadmarks_line;
};

struct RoadMark
{
    RoadMark() = default;

    double s_start = 0;
    double s_end = 0;
    double t_offset = 0;
    double width = 0;

    std::string type = "";
};

} // namespace odr