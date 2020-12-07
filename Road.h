#pragma once

#include "Geometries/CubicSpline.h"
#include "Lanes.h"
#include "Math.hpp"
#include "Utils.hpp"

#include <map>
#include <memory>
#include <set>

namespace odr
{
struct RefLine;

struct Crossfall : public CubicSpline
{
    enum Side
    {
        Both,
        Left,
        Right
    };

    Crossfall() = default;
    double get_crossfall(double s, bool on_left_side) const;

    std::map<double, Side> sides;
};

struct RoadLink
{
    int elementId = -1;

    std::string elementType;
    std::string contactPoint;
};

struct RoadNeighbor
{
    int elementId = -1;

    std::string side;
    std::string direction;
};

struct SpeedRecord
{
    double      max = -1;
    std::string unit;
};

class Road : public std::enable_shared_from_this<Road>
{
public:
    Road(double length, int id, int junction);

    ConstLaneSectionSet get_lanesections() const;
    LaneSectionSet      get_lanesections();

    std::shared_ptr<const LaneSection> get_lanesection(double s) const;
    std::shared_ptr<LaneSection>       get_lanesection(double s);

    std::shared_ptr<const Lane> get_lane(double s, double t, double* t_outer_brdr = nullptr) const;
    std::shared_ptr<Lane>       get_lane(double s, double t, double* t_outer_brdr = nullptr);

    std::map<int, double> get_lane_borders(double s) const;

    Vec3D get_xyz(double s, double t, double z) const;
    Vec3D get_surface_pt(double s, double t) const;
    Mat3D get_transformation_matrix(double s) const;

    int    id;
    int    junction;
    double length;

    RoadLink                  predecessor;
    RoadLink                  successor;
    std::vector<RoadNeighbor> neighbors;

    CubicSpline              lane_offset;
    CubicSpline              superelevation;
    Crossfall                crossfall;
    std::shared_ptr<RefLine> ref_line;

    std::map<double, std::shared_ptr<LaneSection>> s0_to_lanesection;
    std::map<double, std::string>                  s0_to_type;
    std::map<double, SpeedRecord>                  s0_to_speed;
};

using ConstRoadSet = std::set<std::shared_ptr<const Road>, SharedPtrCmp<const Road, int, &Road::id>>;
using RoadSet = std::set<std::shared_ptr<Road>, SharedPtrCmp<Road, int, &Road::id>>;

} // namespace odr