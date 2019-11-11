#pragma once

#include "Geometries.h"
#include "Utils.h"

#include <memory>
#include <map>


struct LaneSection;

struct LaneOffset;

struct ElevationProfile 
{
    ElevationProfile(double s0, double a, double b, double c, double d);
    double get_elevation(double s);
    
    double s0, a, b, c, d;
};

class Road : public std::enable_shared_from_this<Road>
{
    public:
        Road(double length, int id, int junction, std::map<double, std::shared_ptr<RoadGeometry>> geometries );
        void add_lane_section(std::shared_ptr<LaneSection> lane_section);
        Point3D get_refline_point(double s, double t = 0);

        int id;
        double length, junction;
        std::map<double, std::shared_ptr<ElevationProfile>> elevation_profiles;
        std::map<double, std::shared_ptr<RoadGeometry>> geometries;
        std::map<double, std::shared_ptr<LaneSection>> lane_sections;
        std::map<double, std::shared_ptr<LaneOffset>> lane_offsets;
};