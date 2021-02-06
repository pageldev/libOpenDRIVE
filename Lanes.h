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
    Vec3D  get_surface_pt(double s, double t) const;
    Line3D get_border_line(double s_start, double s_end, double eps, bool outer = true) const;
    Mesh3D get_mesh(double s_start, double s_end, double eps) const;

    std::vector<RoadMark> get_roadmarks(double s_start, double s_end) const;
    Mesh3D                get_roadmark_mesh(const RoadMark& roadmark, double eps) const;

    std::set<double> approximate_border_linear(double s_start, double s_end, double eps, bool outer = true) const;

    int  id = 0;
    bool level = false;
    int  predecessor = 0;
    int  successor = 0;

    std::string type;
    CubicSpline lane_width;
    CubicSpline outer_border;
    CubicSpline inner_border;

    std::map<double, HeightOffset>  s_to_height_offset;
    std::map<double, RoadMarkGroup> s_to_roadmark_group;

    std::weak_ptr<Road> road;
};

using ConstLaneSet = std::set<std::shared_ptr<const Lane>, SharedPtrCmp<const Lane, int, &Lane::id>>;
using LaneSet = std::set<std::shared_ptr<Lane>, SharedPtrCmp<Lane, int, &Lane::id>>;

} // namespace odr