#pragma once

#include "Geometries/CubicSpline.h"
#include "Math.hpp"
#include "Mesh.h"
#include "RoadMark.h"
#include "Utils.hpp"
#include "XmlNode.h"

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace odr
{
class Road;
struct LaneSection;

struct HeightOffset
{
    double inner;
    double outer;
};

struct Lane : public XmlNode, public std::enable_shared_from_this<Lane>
{
    Lane(int id, bool level, std::string type);
    virtual ~Lane() = default;

    Vec3D  get_surface_pt(double s, double t, Vec3D* vn = nullptr) const;
    Line3D get_border_line(double s_start, double s_end, double eps, bool outer = true) const;
    Mesh3D get_mesh(double s_start, double s_end, double eps, std::vector<uint32_t>* outline_indices = nullptr) const;

    std::vector<std::shared_ptr<RoadMark>> get_roadmarks(double s_start, double s_end) const;
    Mesh3D                                 get_roadmark_mesh(std::shared_ptr<const RoadMark> roadmark, double eps) const;

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

    std::weak_ptr<Road>        road;
    std::weak_ptr<LaneSection> lane_section;
};

using ConstLaneSet = std::set<std::shared_ptr<const Lane>, SharedPtrCmp<const Lane, int, &Lane::id>>;
using LaneSet = std::set<std::shared_ptr<Lane>, SharedPtrCmp<Lane, int, &Lane::id>>;

} // namespace odr