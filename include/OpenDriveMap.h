#pragma once
#include "Junction.h"
#include "Road.h"
#include "RoadNetworkMesh.h"
#include "RoutingGraph.h"

#include "pugixml.hpp"

#include <map>
#include <string>
#include <vector>

namespace odr
{

class OpenDriveMap
{
public:
    OpenDriveMap(const std::string& xodr_file,
                 const bool         center_map = false,
                 const bool         with_road_objects = true,
                 const bool         with_lateral_profile = true,
                 const bool         with_lane_height = true,
                 const bool         abs_z_for_for_local_road_obj_outline = false,
                 const bool         fix_spiral_edge_cases = true,
                 const bool         with_road_signals = true);

    Road                  get_road(const std::string& id) const;
    std::vector<Road>     get_roads() const;
    Junction              get_junction(const std::string& id) const;
    std::vector<Junction> get_junctions() const;

    RoadNetworkMesh get_road_network_mesh(const double eps) const;
    RoutingGraph    get_routing_graph() const;

    std::string       proj4 = "";
    double            x_offs = 0;
    double            y_offs = 0;
    const std::string xodr_file = "";

    std::map<std::string, Road>     id_to_road;
    std::map<std::string, Junction> id_to_junction;
};

} // namespace odr