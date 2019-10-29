#pragma once

#include "Lanes.h"
#include "Geometries.h"

#include <iostream>
#include <memory>
#include <set>


struct LaneSection;

struct CmpRoadGeometry {
    bool operator()(const std::shared_ptr<RoadGeometry>& lhs, const std::shared_ptr<RoadGeometry>& rhs) const;
};

struct CmpLaneSection {
    bool operator()(const std::shared_ptr<LaneSection>& lhs, const std::shared_ptr<LaneSection>& rhs) const;
};

class Road : public std::enable_shared_from_this<Road>
{
    public:
        Road(double length, int id, int junction, std::set<std::shared_ptr<RoadGeometry>, CmpRoadGeometry> geometries);
        void add_lanesection(std::shared_ptr<LaneSection> lane_section);
        void add_lanesection(std::set<std::shared_ptr<LaneSection>, CmpLaneSection> lane_sections);
        Point3D get_refline_point(double s, double t = 0);

        int id;
        double length, junction;
        std::set<std::shared_ptr<RoadGeometry>, CmpRoadGeometry> geometries;
        std::set<std::shared_ptr<LaneSection>, CmpLaneSection> lane_sections;
};