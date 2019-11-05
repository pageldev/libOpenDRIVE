#pragma once

#include "Geometries.h"
#include "Utils.h"

#include <memory>
#include <set>


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
        Road(double length, int id, int junction, std::set<std::shared_ptr<RoadGeometry>, PtrCompareS0<RoadGeometry>> geometries);
        void add_lane_section(std::shared_ptr<LaneSection> lane_section);
        Point3D get_refline_point(double s, double t = 0);

        int id;
        double length, junction;
        std::set<std::shared_ptr<ElevationProfile>, PtrCompareS0<ElevationProfile>> elevation_profiles;
        std::set<std::shared_ptr<RoadGeometry>, PtrCompareS0<RoadGeometry>> geometries;
        std::set<std::shared_ptr<LaneSection>, PtrCompareS0<LaneSection>> lane_sections;
        std::set<std::shared_ptr<LaneOffset>, PtrCompareS0<LaneOffset>> lane_offsets;
};