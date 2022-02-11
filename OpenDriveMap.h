#pragma once

#include "Mesh.h"
#include "Road.h"
#include "RoadNetworkMesh.h"

#include "pugixml/pugixml.hpp"

#include <map>
#include <memory>
#include <string>

namespace odr
{
class OpenDriveMap
{
public:
    OpenDriveMap(
        std::string xodr_file, bool with_lateralProfile = true, bool with_laneHeight = true, bool center_map = true, bool with_objects = true);

    ConstRoadSet get_roads() const;
    RoadSet      get_roads();

    Mesh3D          get_refline_lines(double eps) const;
    RoadNetworkMesh get_mesh(double eps) const;

    std::string xodr_file = "";
    std::string proj4 = "";

    pugi::xml_document xml_doc;

    double x_offs = 0;
    double y_offs = 0;

    std::map<std::string, std::shared_ptr<Road>> roads;
};

} // namespace odr