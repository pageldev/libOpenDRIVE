#pragma once

#include "Mesh.h"

#include <memory>
#include <vector>

namespace odr
{
class Road;

struct RoadObjectRepeat
{
    double s0 = 0;
    double length = 0;
    double distance = 0;
    double t_start = 0;
    double t_end = 0;
    double width_start = 0;
    double width_end = 0;
    double height_start = 0;
    double height_end = 0;
    double z_offset_start = 0;
    double z_offset_end = 0;
};

struct RoadObjectCorner
{
    double s = 0;
    double t = 0;
    double dz = 0;
    double height = 0;
};

struct RoadObject
{
    RoadObject() = default;

    Mesh3D get_mesh(double eps) const;

    std::string type;
    std::string name;
    std::string id;
    std::string orientation;

    double s0 = 0;
    double t0 = 0;
    double z0 = 0;

    double length = 0;
    double valid_length = 0;
    double width = 0;
    double radius = 0;
    double height = 0;
    double hdg = 0;
    double pitch = 0;
    double roll = 0;

    std::vector<RoadObjectRepeat> repeats;
    std::vector<RoadObjectCorner> outline;

    std::weak_ptr<Road> road;
};
} // namespace odr