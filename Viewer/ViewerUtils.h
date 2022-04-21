#pragma once
#include "Mesh.h"
#include "RoadNetworkMesh.h"

namespace odr
{

class OpenDriveMap;

Mesh3D          get_refline_segments(const OpenDriveMap& odr_map, double eps);
RoadNetworkMesh get_road_network_mesh(const OpenDriveMap& odr_map, double eps);

} // namespace odr