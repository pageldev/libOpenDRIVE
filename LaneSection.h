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

    double get_end() const;

    ConstLaneSet get_lanes() const;
    LaneSet      get_lanes();

    std::shared_ptr<const Lane> get_lane(double s, double t) const;
    std::shared_ptr<Lane>       get_lane(double s, double t);

    // std::vector<RoadMarkLines> get_roadmark_lines(int lane_id, double resolution) const;
    // std::vector<RoadMarkLines> get_roadmark_lines(double resolution) const;

    double              s0 = 0;
    std::weak_ptr<Road> road;

    std::map<int, std::shared_ptr<Lane>> id_to_lane;
};

using ConstLaneSectionSet = std::set<std::shared_ptr<const LaneSection>, SharedPtrCmp<const LaneSection, double, &LaneSection::s0>>;
using LaneSectionSet = std::set<std::shared_ptr<LaneSection>, SharedPtrCmp<LaneSection, double, &LaneSection::s0>>;

} // namespace odr
