#include "Road.h"


Road::Road(double length, int id, int junction, std::vector<std::shared_ptr<RoadGeometry>> geometries)
    : length(length), id(id), junction(junction), geometries(geometries)
{  
    std::sort(this->geometries.begin(), this->geometries.end()
        , [](const std::shared_ptr<RoadGeometry>& a, const std::shared_ptr<RoadGeometry>& b) { return a->s0 < b->s0; } );
}

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
    std::shared_ptr<RoadGeometry> target_geom = this->geometries.front();
    for( int idx = 1; idx < geometries.size(); idx++ ) {
        if( geometries.at(idx)->s0 > s ) {
            break;
        } else {
            target_geom = geometries.at(idx);
        }
    }
    return target_geom->get_point(s, t);
}