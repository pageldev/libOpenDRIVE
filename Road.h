#pragma once

#include "Geometries/Geometries.h"
#include "Utils.hpp"

#include <map>
#include <memory>

namespace odr
{

struct LaneSection;

struct LaneOffset;

struct ElevationProfile
{
    ElevationProfile(double s0, double a, double b, double c, double d);
    double get_elevation(const double s) const;

    double s0, a, b, c, d;
};

class Road : public std::enable_shared_from_this<Road>
{
public:
    Road(double length, int id, int junction);
    void   add_lane_section(std::shared_ptr<LaneSection> lane_section);
    Vec3D  get_refline_point(const double s, const double t = 0, const bool with_offset = false) const;
    double get_elevation(const double s) const;
    double project(double x, double y) const;

    int    id;
    double length, junction;

    std::map<double, std::shared_ptr<ElevationProfile>> elevation_profiles;
    std::map<double, std::shared_ptr<RoadGeometry>>     geometries;
    std::map<double, std::shared_ptr<LaneSection>>      lane_sections;
    std::map<double, std::shared_ptr<LaneOffset>>       lane_offsets;
};

} // namespace odr