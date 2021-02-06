#pragma once

#include "Road.h"
#include "RoadNetworkMesh.h"

#include <map>
#include <memory>
#include <string>

namespace odr
{
class OpenDriveMap
{
public:
    OpenDriveMap(std::string xodr_file, bool with_lateralProfile = true, bool with_laneHeight = true);

    ConstRoadSet get_roads() const;
    RoadSet      get_roads();

    Mesh3D          get_refline_segments(double eps) const;
    RoadNetworkMesh get_mesh(double eps) const;

    std::string xodr_file;
    std::string proj4;

    std::map<std::string, std::shared_ptr<Road>> roads;
};

} // namespace odr