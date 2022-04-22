#pragma once
#include "Junction.h"
#include "Road.h"
#include "RoutingGraph.h"

#include <pugixml/pugixml.hpp>

#include <map>
#include <string>
#include <vector>

namespace odr
{

struct OpenDriveMapConfig
{
    bool with_lateralProfile = true;
    bool with_laneHeight = true;
    bool with_road_objects = true;
    bool center_map = true;
    bool abs_z_for_for_local_road_obj_outline = false;
};

class OpenDriveMap
{
public:
    OpenDriveMap(const std::string& xodr_file, const OpenDriveMapConfig& config = OpenDriveMapConfig{});

    std::vector<Road>     get_roads() const;
    std::vector<Junction> get_junctions() const;

    RoutingGraph get_routing_graph() const;

    std::string        proj4 = "";
    double             x_offs = 0;
    double             y_offs = 0;
    const std::string  xodr_file = "";
    pugi::xml_document xml_doc;

    std::map<std::string, Road>     id_to_road;
    std::map<std::string, Junction> id_to_junction;
};

} // namespace odr