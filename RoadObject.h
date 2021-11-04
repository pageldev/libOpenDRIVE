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

struct RoadObjectCornerLocal
{
    double u = 0;
    double v = 0;
    double z = 0;
    double height = 0;
};

struct RoadObject
{
    RoadObject() = default;

    Mesh3D get_mesh(double eps) const;

    static Mesh3D get_cylinder(double eps, double radius, double height);
    static Mesh3D get_box(double width, double length, double height);

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

    std::vector<RoadObjectCornerLocal> local_outline;

    std::weak_ptr<Road> road;
};
} // namespace odr