#pragma once
#include "Road.h"
#include "RoutingGraph.h"

#include <pugixml/pugixml.hpp>

#include <map>
#include <memory>
#include <string>

namespace odr
{

class Junction;

struct OpenDriveMapConfig
{
    bool with_lateralProfile = true;
    bool with_laneHeight = true;
    bool with_road_objects = true;
    bool center_map = true;
    bool abs_z_for_for_local_road_obj_outline = true;
};

class OpenDriveMap
{
public:
    OpenDriveMap(const std::string& xodr_file, const OpenDriveMapConfig& config = OpenDriveMapConfig{});

    ConstRoadSet get_roads() const;
    RoadSet      get_roads();

    RoutingGraph get_routing_graph() const;

    const std::string  xodr_file = "";
    std::string        proj4 = "";
    pugi::xml_document xml_doc;

    double x_offs = 0;
    double y_offs = 0;

    std::map<std::string, std::shared_ptr<Road>>     roads;
    std::map<std::string, std::shared_ptr<Junction>> junctions;
};

} // namespace odr