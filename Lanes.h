#pragma once

#include "Geometries/CubicSpline.h"
#include "Math.hpp"
#include "RoadMark.h"
#include "Utils.hpp"

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace odr
{
class Road;

struct HeightOffset
{
    double inner;
    double outer;
};

struct Lane : public std::enable_shared_from_this<Lane>
{
    Lane(int id, bool level, std::string type);
    Vec3D get_surface_pt(double s, double t) const;

    int  id = 0;
    bool level = false;
    int  predecessor = 0;
    int  successor = 0;

    std::string type;
    CubicSpline lane_width;
    CubicSpline outer_border;
    CubicSpline inner_border;

    std::map<double, HeightOffset> s_to_height_offset;
    std::map<double, RoadMark>     s_to_roadmark;

    std::weak_ptr<Road> road;
};

struct LaneLines
{
    LaneLines() = default;
    Mesh3D generate_mesh() const;

    std::shared_ptr<Lane> lane = nullptr;

    Line3D innner_border;
    Line3D outer_border;
};

using ConstLaneSet = std::set<std::shared_ptr<const Lane>, SharedPtrCmp<const Lane, int, &Lane::id>>;
using LaneSet = std::set<std::shared_ptr<Lane>, SharedPtrCmp<Lane, int, &Lane::id>>;

} // namespace odr