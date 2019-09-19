#pragma once

#include "Lanes.h"
#include "Geometries.h"

#include <memory>
#include <vector>
#include <iostream>


struct LaneSection;

class Road : public std::enable_shared_from_this<Road>
{
    public:
        Road(double length, int id, int junction, std::vector<std::shared_ptr<RoadGeometry>> geometries);
        void add_lanesection(std::shared_ptr<LaneSection> lane_section);
        void add_lanesection(std::vector<std::shared_ptr<LaneSection>> lane_sections);
        std::pair<double, double> get_refline_point(double s, double t = 0);

        double length, id, junction;
        std::vector<std::shared_ptr<RoadGeometry>> geometries;
        std::vector<std::shared_ptr<LaneSection>> lane_sections;
};