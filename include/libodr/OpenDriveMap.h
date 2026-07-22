#pragma once
#include "libodr/Junction.h"
#include "libodr/Lane.h"
#include "libodr/Road.h"
#include "libodr/RoadNetworkMesh.h"
#include "libodr/RoutingGraph.h"

#include "pugixml.hpp"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace odr
{

struct OpenDriveMapHeader
{
    OpenDriveMapHeader(std::optional<int>         rev_major = std::nullopt,
                       std::optional<int>         rev_minor = std::nullopt,
                       std::optional<double>      north = std::nullopt,
                       std::optional<double>      east = std::nullopt,
                       std::optional<double>      south = std::nullopt,
                       std::optional<double>      west = std::nullopt,
                       std::optional<std::string> date = std::nullopt,
                       std::optional<std::string> name = std::nullopt,
                       std::optional<std::string> vendor = std::nullopt,
                       std::optional<std::string> version = std::nullopt,
                       std::optional<std::string> proj = std::nullopt);

    std::optional<int> rev_major;
    std::optional<int> rev_minor;

    std::optional<double> north;
    std::optional<double> east;
    std::optional<double> south;
    std::optional<double> west;

    std::optional<std::string> date;
    std::optional<std::string> name;
    std::optional<std::string> vendor;
    std::optional<std::string> version;
    std::optional<std::string> proj;
};

struct XodrParseError
{
    pugi::xml_node node;
    std::string    description;
};

struct XodrParseResult
{
    std::vector<XodrParseError> errors;
};

class OpenDriveMap
{
public:
    OpenDriveMap() = default;

    XodrParseResult load(const pugi::xml_document& xml_doc,
                         const bool                with_road_objects = true,
                         const bool                with_lateral_profile = true,
                         const bool                with_lane_height = true,
                         const bool                abs_z_for_for_local_road_obj_outline = false,
                         const bool                fix_spiral_edge_cases = true,
                         const bool                with_road_signals = true,
                         const bool                treat_value_zero_as_missing = true);

    void reset();

    Road                  get_road(const std::string& id) const;
    std::vector<Road>     get_roads() const;
    Junction              get_junction(const std::string& id) const;
    std::vector<Junction> get_junctions() const;

    RoadNetworkMesh get_road_network_mesh(const double eps) const;
    RoutingGraph    get_routing_graph() const;

    OpenDriveMapHeader header;

    std::map<std::string, Road>     id_to_road;
    std::map<std::string, Junction> id_to_junction;
};

} // namespace odr
