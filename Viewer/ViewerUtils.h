#pragma once

#include "Mesh.h"
#include "OpenDriveMap.h"
#include "RoadNetworkMesh.h"

namespace odr
{

Mesh3D          get_refline_segments(const OpenDriveMap& odr_map, double eps);
RoadNetworkMesh get_road_network_mesh(const OpenDriveMap& odr_map, double eps);

} // namespace odr