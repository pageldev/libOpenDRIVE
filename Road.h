#pragma once

#include "Geometries/Geometries.h"
#include "RefLine.h"
#include "Utils.hpp"

#include <map>
#include <memory>

namespace odr
{

struct LaneSection;

struct LaneOffset;

class Road : public std::enable_shared_from_this<Road>
{
public:
    Road(double length, int id, int junction);
    void add_lane_section(std::shared_ptr<LaneSection> lane_section);
    double get_lane_offset(double s) const;

    int    id, junction;
    double length;

    std::shared_ptr<RefLine> ref_line;

    std::map<double, std::shared_ptr<LaneSection>> lane_sections;
    std::map<double, std::shared_ptr<LaneOffset>>  lane_offsets;
};

} // namespace odr