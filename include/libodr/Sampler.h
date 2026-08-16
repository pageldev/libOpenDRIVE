#pragma once
#include "libodr/Math.hpp"

#include <set>

namespace odr
{

class Road;
struct CubicBounds;
struct Lane;
struct RefLine;
struct RoadGeometry;

struct DerivativeBounds
{
    double value = 0;
    double d1 = 0;
    double d2 = 0;
};

struct GeometryBounds
{
    double position_d1 = 1;
    double position_d2 = 0;
    double speed_lower = 1; // parameter-relative speed; 1 for arc-length geometries (line/arc/spiral)
    double tangent_d1 = 0;  // change in direction and paramter-speed; equals |pos''(s)| for arc-length geometries
    double tangent_d2 = 0;
    double lateral_d1 = 0; // change in lateral direction; equals |pos''(s)| for arc-length geometries
    double lateral_d2 = 0;
};

struct EdgePoints
{
    Vec3D inner;
    Vec3D outer;
};

DerivativeBounds tan_bounds(const CubicBounds& angle);
DerivativeBounds product(const DerivativeBounds& lhs, const DerivativeBounds& rhs);
DerivativeBounds sum(const DerivativeBounds& lhs, const DerivativeBounds& rhs);
DerivativeBounds magnitudes(const CubicBounds& value, bool absolute_value);

GeometryBounds geometry_bounds(const RoadGeometry& geometry, double s_start, double s_end);

std::set<double> get_ref_line_mandatory_s_samples(const RefLine& ref_line, double s_start, double s_end);
std::set<double> get_road_mandatory_s_samples(const Road& road, double s_start, double s_end);
std::set<double> get_lane_mandatory_s_samples(const Road& road, const Lane& lane, const Lane& inner_lane, double s_start, double s_end);

struct RefLineSampler
{
    explicit RefLineSampler(const RefLine& ref_line);

    std::set<double> get_polyline_s_samples(double s_start, double s_end, double eps) const;

    const RefLine& ref_line;

private:
    void refine_interval(std::set<double>& samples, double s_start, double s_end, double eps) const;
};

struct RoadSampler
{
    explicit RoadSampler(const Road& road);

    std::set<double> get_polyline_s_samples(double s_start, double t_start, double s_end, double t_end, double eps) const;

    const Road& road;

private:
    void refine_interval(std::set<double>& samples, double s_start, double t_start, double s_end, double t_end, double eps) const;
};

struct LaneSampler
{
    LaneSampler(const Road& road, double lane_section_s, const Lane& lane, const Lane& inner_lane);

    EdgePoints edge_points(double s) const;

    std::set<double> get_mesh_s_samples(double s_start, double s_end, double eps, bool allow_extrapolate = true) const;
    std::set<double> get_border_s_samples(double s_start, double s_end, double t_offset, double eps, bool allow_extrapolate = true) const;

    const Road&  road;
    const double lane_section_s;
    const Lane&  lane;
    const Lane&  inner_lane;

private:
    void refine_interval(std::set<double>& samples, double s_start, double s_end, const EdgePoints& start, const EdgePoints& end, double eps) const;
    void refine_border_interval(std::set<double>& samples, double s_start, double s_end, double t_offset, double eps) const;
};

} // namespace odr
