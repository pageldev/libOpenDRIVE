#include "Road.h"


Road::Road(double length, int id, int junction, std::vector<std::shared_ptr<RoadGeometry>> geometries)
    : length(length), id(id), junction(junction), geometries(geometries)
{  }

void Road::add_lanesection(std::shared_ptr<LaneSection> lane_section)
{
    this->lane_sections.push_back(lane_section);
    lane_section->road = shared_from_this();
}

void Road::add_lanesection(std::vector<std::shared_ptr<LaneSection>> lane_sections)
{
    for( std::shared_ptr<LaneSection> lane_section : lane_sections ) {
        this->add_lanesection(lane_section);
    }
}

std::pair<double, double> Road::get_refline_point(double s, double t)
{
    for( std::shared_ptr<RoadGeometry> geom : geometries ) {
        std::cout << geom->s0 << std::endl;
    }
    return std::make_pair(0.0, 0.0);
}