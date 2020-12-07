#pragma once

#include "Geometries/CubicSpline.h"
#include "Math.hpp"
#include "Utils.hpp"

#include <map>
#include <memory>
#include <set>
#include <string>

namespace odr
{
class Road;
struct LaneSection;

struct LaneVertices
{
    std::vector<Vec3D>  vertices;
    std::vector<size_t> indices;

    int         lane_id;
    std::string type;
};

struct HeightOffset
{
    double inner;
    double outer;
};

struct Lane : public std::enable_shared_from_this<Lane>
{
    Lane(int id, bool level, std::string type);

    int  id;
    bool level = false;
    int  predecessor = 0;
    int  successor = 0;

    std::string type;
    CubicSpline lane_width;

    std::map<double, HeightOffset> s0_to_height_offset;
};

using ConstLaneSet = std::set<std::shared_ptr<const Lane>, SharedPtrCmp<const Lane, int, &Lane::id>>;
using LaneSet = std::set<std::shared_ptr<Lane>, SharedPtrCmp<Lane, int, &Lane::id>>;

struct LaneSection : public std::enable_shared_from_this<LaneSection>
{
    LaneSection(double s0);

    ConstLaneSet get_lanes() const;
    LaneSet      get_lanes();

    std::shared_ptr<const Lane> get_lane(double s, double t, double* t_outer_brdr = nullptr) const;
    std::shared_ptr<Lane>       get_lane(double s, double t, double* t_outer_brdr = nullptr);

    std::map<int, std::vector<Vec3D>> get_lane_outlines(double resolution) const;
    std::vector<LaneVertices>         get_lane_vertices(double resolution) const;

    double              s0;
    std::weak_ptr<Road> road;

    std::map<int, std::shared_ptr<Lane>> id_to_lane;
};

using ConstLaneSectionSet = std::set<std::shared_ptr<const LaneSection>, SharedPtrCmp<const LaneSection, double, &LaneSection::s0>>;
using LaneSectionSet = std::set<std::shared_ptr<LaneSection>, SharedPtrCmp<LaneSection, double, &LaneSection::s0>>;

} // namespace odr