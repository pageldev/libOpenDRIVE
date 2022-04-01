#pragma once

#include "Math.hpp"
#include "Mesh.h"
#include "Utils.hpp"
#include "XmlNode.h"

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace odr
{
class Road;

struct RoadObjectRepeat : public XmlNode
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

struct RoadObjectCorner : public XmlNode
{
    enum class Type
    {
        Local_RelZ, // z relative to road’s reference line
        Local_AbsZ, // absolute z value
        Road
    };

    Vec3D  pt;
    double height = 0;
    Type   type = Type::Road;
};

struct RoadObject : public XmlNode
{
    RoadObject() = default;

    Mesh3D get_mesh(double eps) const;

    static Mesh3D get_cylinder(double eps, double radius, double height);
    static Mesh3D get_box(double width, double length, double height);

    std::string id;
    std::string type;
    std::string name;
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

using ConstRoadObjectSet = std::set<std::shared_ptr<const RoadObject>, SharedPtrCmp<const RoadObject, std::string, &RoadObject::id>>;
using RoadObjectSet = std::set<std::shared_ptr<RoadObject>, SharedPtrCmp<RoadObject, std::string, &RoadObject::id>>;

} // namespace odr