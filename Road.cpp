#include "Road.h"


bool CmpRoadGeometry::operator()(const std::shared_ptr<RoadGeometry>& lhs, const std::shared_ptr<RoadGeometry>& rhs) const {
    return (lhs->s0 < rhs->s0);
}

bool CmpLaneSection::operator()(const std::shared_ptr<LaneSection>& lhs, const std::shared_ptr<LaneSection>& rhs) const {
    return (lhs->s0 < rhs->s0);
}

Road::Road(double length, int id, int junction, std::set<std::shared_ptr<RoadGeometry>, CmpRoadGeometry> geometries)
    : length(length), id(id), junction(junction), geometries(geometries)
{  }

void Road::add_lanesection(std::shared_ptr<LaneSection> lane_section)
{
    this->lane_sections.insert(lane_section);
    lane_section->road = shared_from_this();
}

void Road::add_lanesection(std::set<std::shared_ptr<LaneSection>, CmpLaneSection> lane_sections)
{
    for( std::shared_ptr<LaneSection> lane_section : lane_sections ) {
        this->add_lanesection(lane_section);
    }
}

Point3D Road::get_refline_point(double s, double t)
{
    std::set<std::shared_ptr<RoadGeometry>, CmpRoadGeometry>::iterator target_geom_iter 
        = this->geometries.upper_bound(std::make_shared<Line>(s, 0.0, 0.0, 0.0, 0.0));
    if( target_geom_iter != geometries.begin() ) {
        target_geom_iter--;
    }
    return (*target_geom_iter)->get_point(s, t);
}