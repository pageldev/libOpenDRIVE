#pragma once

#include "Lanes.h"

#include "Math.hpp"
#include "Utils.hpp"

#include <map>
#include <memory>
#include <set>
#include <string>

namespace odr
{
class Road;

struct LaneVertices
{
    std::vector<Vec3D>  vertices;
    std::vector<size_t> indices;

    int         lane_id;
    std::string type;
};

struct LaneSection : public std::enable_shared_from_this<LaneSection>
{
    LaneSection(double s0);

    ConstLaneSet get_lanes() const;
    LaneSet      get_lanes();

    std::shared_ptr<const Lane> get_lane(double s, double t, double* t_outer_brdr = nullptr) const;
    std::shared_ptr<Lane>       get_lane(double s, double t, double* t_outer_brdr = nullptr);

    std::map<int, std::pair<Line3D, Line3D>>
                              get_lane_border_lines(double resolution, bool with_lateralProfile = true, bool with_laneHeight = true) const;
    std::vector<LaneVertices> get_lane_vertices(double resolution, bool with_lateralProfile = true, bool with_laneHeight = true) const;

    double              s0;
    std::weak_ptr<Road> road;

    std::map<int, std::shared_ptr<Lane>> id_to_lane;
};

using ConstLaneSectionSet = std::set<std::shared_ptr<const LaneSection>, SharedPtrCmp<const LaneSection, double, &LaneSection::s0>>;
using LaneSectionSet = std::set<std::shared_ptr<LaneSection>, SharedPtrCmp<LaneSection, double, &LaneSection::s0>>;

} // namespace odr
