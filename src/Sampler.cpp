#include "libodr/Sampler.h"

#include "libodr/Geometries/Arc.h"
#include "libodr/Geometries/CubicSpline.h"
#include "libodr/Geometries/Line.h"
#include "libodr/Geometries/ParamPoly3.h"
#include "libodr/Geometries/Spiral.h"
#include "libodr/Road.h"
#include "libodr/Utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace odr
{

namespace
{

struct IntervalBounds
{
    CubicBounds outer;
    CubicBounds inner;
    CubicBounds width;
    CubicBounds theta;

    DerivativeBounds crossfall;

    double inner_height_offset = 0;
    double outer_height_offset = 0;
    double ref_line_d1 = 0;
    double ref_line_d2 = 0;
    double frame_d1 = 0;
    double frame_d2 = 0;
    double s_length = 0;
};

struct RefLineIntervalBounds
{
    IntervalBounds interval;
    CubicBounds    elevation;
    GeometryBounds geometry;
};

// precondition: s-interval is a breakpoint-free interval (single geometry etc..)
RefLineIntervalBounds get_ref_line_interval_bounds(const RefLine& ref_line, double s_start, double s_end)
{
    const double                   s_mid = 0.5 * (s_start + s_end);
    const std::optional<CubicPoly> elevation_poly = ref_line.elevation_profile.get_poly(s_mid);
    const RoadGeometry*            road_geometry = ref_line.get_geometry(s_mid);
    require_or_throw(road_geometry != nullptr, "unsupported reference line geometry");

    RefLineIntervalBounds bounds;
    bounds.elevation = elevation_poly ? elevation_poly->bounds(s_start, s_end) : CubicBounds{};
    bounds.geometry = geometry_bounds(*road_geometry, s_start, s_end);
    bounds.interval.ref_line_d1 = std::hypot(bounds.geometry.position_d1, bounds.elevation.d1);
    bounds.interval.ref_line_d2 = std::hypot(bounds.geometry.position_d2, bounds.elevation.d2);
    bounds.interval.s_length = s_end - s_start;
    return bounds;
}

// precondition: s-interval is a breakpoint-free interval (single geometry etc..)
IntervalBounds get_road_interval_bounds(const Road& road, double s_start, double s_end)
{
    const double                   s_mid = 0.5 * (s_start + s_end);
    const std::optional<CubicPoly> superelevation_poly = road.superelevation.get_poly(s_mid);

    const RefLineIntervalBounds ref_line_bounds = get_ref_line_interval_bounds(road.ref_line, s_start, s_end);
    IntervalBounds              interval = ref_line_bounds.interval;
    interval.theta = superelevation_poly ? superelevation_poly->bounds(s_start, s_end) : CubicBounds{};

    const CubicBounds&    elevation = ref_line_bounds.elevation;
    const GeometryBounds& geometry = ref_line_bounds.geometry;

    const double tangent_d1 = std::hypot(geometry.tangent_d1, elevation.d2);
    const double tangent_d2 = std::hypot(geometry.tangent_d2, elevation.d3);
    const double es_d1 = tangent_d1 / geometry.speed_lower;
    const double es_d2 = tangent_d2 / geometry.speed_lower + 3 * tangent_d1 * tangent_d1 / (geometry.speed_lower * geometry.speed_lower);
    const double base_d1 = es_d1 + geometry.lateral_d1;
    const double base_d2 = es_d2 + geometry.lateral_d2 + 2 * es_d1 * geometry.lateral_d1;
    interval.frame_d1 = interval.theta.d1 + base_d1;
    interval.frame_d2 = base_d2 + 2 * interval.theta.d1 * base_d1 + interval.theta.d2 + interval.theta.d1 * interval.theta.d1;
    return interval;
}

// precondition: s-interval is a breakpoint-free interval (single geometry etc..)
IntervalBounds get_lane_interval_bounds(const Road& road, const Lane& lane, const Lane& inner_lane, double s_start, double s_end)
{
    const double s_mid = 0.5 * (s_start + s_end);

    const std::optional<CubicPoly> outer_poly = lane.outer_border.get_poly(s_mid);
    const std::optional<CubicPoly> inner_poly = inner_lane.outer_border.get_poly(s_mid);

    CubicPoly width_poly = outer_poly.value_or(CubicPoly{});
    if (inner_poly)
        width_poly.subtract(*inner_poly);

    IntervalBounds interval = get_road_interval_bounds(road, s_start, s_end);
    interval.outer = outer_poly ? outer_poly->bounds(s_start, s_end) : CubicBounds{};
    interval.inner = inner_poly ? inner_poly->bounds(s_start, s_end) : CubicBounds{};
    interval.width = width_poly.bounds(s_start, s_end);

    const auto crossfall_record_iter = road.crossfall.records.upper_bound(s_mid); // first element > s_mid
    if (crossfall_record_iter != road.crossfall.records.begin())
    {
        const Crossfall::Record& record = std::prev(crossfall_record_iter)->second;
        const bool excluded = (lane.id > 0 && record.side == Crossfall::Side::Right) || (lane.id < 0 && record.side == Crossfall::Side::Left);
        if (!excluded)
            interval.crossfall = tan_bounds(record.poly.bounds(s_start, s_end));
    }

    const auto height_record_iter = lane.s_to_height_offset.upper_bound(s_mid);
    if (height_record_iter != lane.s_to_height_offset.begin())
    {
        interval.inner_height_offset = std::prev(height_record_iter)->second.inner;
        interval.outer_height_offset = std::prev(height_record_iter)->second.outer;
    }

    return interval;
}

double max_abs(const CubicBounds& bounds)
{
    return std::max(std::abs(bounds.min), std::abs(bounds.max));
}

double curve_error(const IntervalBounds& interval, const CubicBounds& lateral, const DerivativeBounds& height)
{
    const double speed = interval.ref_line_d1 + lateral.d1 + max_abs(lateral) * interval.frame_d1 + height.d1 + height.value * interval.frame_d1;
    const double acceleration = interval.ref_line_d2 + lateral.d2 + 2 * lateral.d1 * interval.frame_d1 + max_abs(lateral) * interval.frame_d2 +
                                height.d2 + 2 * height.d1 * interval.frame_d1 + height.value * interval.frame_d2;
    // A curve is within min(length * max|p'| / 2, length^2 * max|p''| / 8) of its chord.
    return std::min(0.5 * interval.s_length * speed, 0.125 * interval.s_length * interval.s_length * acceleration);
}

} // namespace

DerivativeBounds tan_bounds(const CubicBounds& angle)
{
    const double first_pole = 0.5 * M_PI + std::ceil((angle.min - 0.5 * M_PI) / M_PI) * M_PI;
    if (first_pole <= angle.max)
        return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    const double value = std::max(std::abs(std::tan(angle.min)), std::abs(std::tan(angle.max)));
    const double sec_squared = 1 + value * value;
    return {value, sec_squared * angle.d1, sec_squared * angle.d2 + 2 * sec_squared * value * angle.d1 * angle.d1};
}

DerivativeBounds product(const DerivativeBounds& lhs, const DerivativeBounds& rhs)
{
    return {lhs.value * rhs.value, lhs.d1 * rhs.value + lhs.value * rhs.d1, lhs.d2 * rhs.value + 2 * lhs.d1 * rhs.d1 + lhs.value * rhs.d2};
}

DerivativeBounds sum(const DerivativeBounds& lhs, const DerivativeBounds& rhs)
{
    return {lhs.value + rhs.value, lhs.d1 + rhs.d1, lhs.d2 + rhs.d2};
}

DerivativeBounds magnitudes(const CubicBounds& value, bool absolute_value)
{
    const bool crosses_zero = value.min < 0 && value.max > 0;
    return {max_abs(value), value.d1, absolute_value && crosses_zero ? std::numeric_limits<double>::infinity() : value.d2};
}

GeometryBounds geometry_bounds(const RoadGeometry& geometry, double s_start, double s_end)
{
    GeometryBounds bounds{1, 0, 1, 0, 0, 0, 0};

    if (dynamic_cast<const Line*>(&geometry))
    {
        return bounds;
    }
    else if (const Arc* arc = dynamic_cast<const Arc*>(&geometry))
    {
        bounds.position_d2 = bounds.tangent_d1 = bounds.lateral_d1 = std::abs(arc->curvature);
        bounds.tangent_d2 = bounds.lateral_d2 = arc->curvature * arc->curvature;
        return bounds;
    }
    else if (const Spiral* spiral = dynamic_cast<const Spiral*>(&geometry))
    {
        bounds.position_d2 = bounds.tangent_d1 = bounds.lateral_d1 = std::max(std::abs(spiral->curv_start), std::abs(spiral->curv_end));
        bounds.tangent_d2 = bounds.lateral_d2 = std::abs(spiral->c_dot) + bounds.lateral_d1 * bounds.lateral_d1;
        return bounds;
    }
    else if (const ParamPoly3* param_poly = dynamic_cast<const ParamPoly3*>(&geometry))
    {
        const double p_start = param_poly->cubic_bezier.get_t(s_start - param_poly->s);
        const double p_end = param_poly->cubic_bezier.get_t(s_end - param_poly->s);
        const double dp_ds = (p_end - p_start) / (s_end - s_start);

        const CubicBounds u = CubicPoly(param_poly->aU, param_poly->bU, param_poly->cU, param_poly->dU).bounds(p_start, p_end);
        const CubicBounds v = CubicPoly(param_poly->aV, param_poly->bV, param_poly->cV, param_poly->dV).bounds(p_start, p_end);

        const double position_d1_p_bound = std::hypot(u.d1, v.d1);
        const double position_d2_p_bound = std::hypot(u.d2, v.d2);
        const double position_d3_p_bound = std::hypot(u.d3, v.d3);
        const double u_d1_p_min_abs = (u.d1_min <= 0 && u.d1_max >= 0) ? 0 : std::min(std::abs(u.d1_min), std::abs(u.d1_max));
        const double v_d1_p_min_abs = (v.d1_min <= 0 && v.d1_max >= 0) ? 0 : std::min(std::abs(v.d1_min), std::abs(v.d1_max));
        const double position_d1_p_lower_bound = std::hypot(u_d1_p_min_abs, v_d1_p_min_abs);

        bounds.position_d1 = position_d1_p_bound * dp_ds; // make valid for s-range
        bounds.position_d2 = position_d2_p_bound * dp_ds * dp_ds;
        bounds.speed_lower = position_d1_p_lower_bound;
        bounds.tangent_d1 = position_d2_p_bound * dp_ds;
        bounds.tangent_d2 = position_d3_p_bound * dp_ds * dp_ds;
        bounds.lateral_d1 = bounds.tangent_d1 / bounds.speed_lower;
        bounds.lateral_d2 =
            bounds.tangent_d2 / bounds.speed_lower + 3 * bounds.tangent_d1 * bounds.tangent_d1 / (bounds.speed_lower * bounds.speed_lower);
        return bounds;
    }
    else
    {
        throw std::runtime_error("unsupported reference line geometry");
    }

    return bounds;
}

std::set<double> get_ref_line_mandatory_s_samples(const RefLine& ref_line, double s_start, double s_end)
{
    std::set<double> samples{s_start, s_end};

    const auto insert_map_keys = [&](const auto& map)
    {
        for (const auto& [s, _] : map)
        {
            if (s > s_start && s < s_end)
                samples.insert(s);
        }
    };

    insert_map_keys(ref_line.s_to_geometry);
    insert_map_keys(ref_line.elevation_profile.s_to_poly);

    // paramPoly3 uses a piecewise-linear arc-length-to-parameter map, include those breakpoints
    for (const auto& [_, geometry] : ref_line.s_to_geometry)
    {
        const auto* param_poly3 = dynamic_cast<const ParamPoly3*>(geometry.get());
        if (!param_poly3)
            continue;
        for (const auto& [arclen, __] : param_poly3->cubic_bezier.arclen_t)
        {
            const double s = param_poly3->s + arclen;
            if (s > s_start && s < s_end)
                samples.insert(s);
        }
    }

    return samples;
}

std::set<double> get_road_mandatory_s_samples(const Road& road, double s_start, double s_end)
{
    std::set<double> samples = get_ref_line_mandatory_s_samples(road.ref_line, s_start, s_end);

    if (s_start < 0.0 && s_end > 0.0)
        samples.insert(0.0);
    if (s_start < road.length && s_end > road.length)
        samples.insert(road.length);

    for (const auto& [s, _] : road.superelevation.s_to_poly)
    {
        if (s > s_start && s < s_end)
            samples.insert(s);
    }

    return samples;
}

std::set<double> get_lane_mandatory_s_samples(const Road& road, const Lane& lane, const Lane& inner_lane, double s_start, double s_end)
{
    std::set<double> samples = get_road_mandatory_s_samples(road, s_start, s_end);

    const auto insert_map_keys = [&](const auto& map) -> void
    {
        for (const auto& [s, _] : map)
        {
            if (s > s_start && s < s_end)
                samples.insert(s);
        }
    };

    insert_map_keys(road.crossfall.records);
    insert_map_keys(lane.outer_border.s_to_poly);
    insert_map_keys(inner_lane.outer_border.s_to_poly);
    insert_map_keys(lane.s_to_height_offset);

    return samples;
}

RefLineSampler::RefLineSampler(const RefLine& ref_line) : ref_line(ref_line) {}

std::set<double> RefLineSampler::get_polyline_s_samples(double s_start, double s_end, double eps) const
{
    require_or_throw(std::isfinite(eps) && eps > 0, "eps must be finite and greater than 0 (got {})", eps);
    require_or_throw(std::isfinite(s_start) && std::isfinite(s_end) && s_start < s_end, "invalid reference line range [{}, {}]", s_start, s_end);

    std::set<double>          samples = get_ref_line_mandatory_s_samples(ref_line, s_start, s_end);
    const std::vector<double> mandatory_samples(samples.begin(), samples.end());
    for (std::size_t idx = 1; idx < mandatory_samples.size(); idx++)
        refine_interval(samples, mandatory_samples[idx - 1], mandatory_samples[idx], eps);

    return samples;
}

void RefLineSampler::refine_interval(std::set<double>& samples, double s_start, double s_end, double eps) const
{
    const IntervalBounds interval = get_ref_line_interval_bounds(ref_line, s_start, s_end).interval;
    const double         error = curve_error(interval, CubicBounds{}, DerivativeBounds{});
    if (std::isfinite(error) && std::nextafter(error, std::numeric_limits<double>::infinity()) <= eps)
        return;

    const double s_mid = 0.5 * (s_start + s_end);
    require_or_throw(s_mid > s_start && s_mid < s_end, "eps {} is too small to represent reference line", eps);
    samples.insert(s_mid);
    refine_interval(samples, s_start, s_mid, eps);
    refine_interval(samples, s_mid, s_end, eps);
}

RoadSampler::RoadSampler(const Road& road) : road(road) {}

std::set<double> RoadSampler::get_polyline_s_samples(double s_start, double t_start, double s_end, double t_end, double eps) const
{
    require_or_throw(std::isfinite(eps) && eps > 0, "eps must be finite and greater than 0 (got {})", eps);
    require_or_throw(std::isfinite(t_start) && std::isfinite(t_end), "invalid t-range [{}, {}]", t_start, t_end);
    require_or_throw(std::isfinite(s_start) && std::isfinite(s_end) && s_start < s_end, "invalid s-range [{}, {}]", s_start, s_end);

    const double t_slope = (t_end - t_start) / (s_end - s_start);
    const auto   get_t = [&](double s) { return t_start + (s - s_start) * t_slope; };

    std::set<double> samples{s_start, s_end};
    const double     refinement_start = std::max(s_start, 0.0);
    const double     refinement_end = std::min(s_end, road.length);
    if (refinement_start < refinement_end)
    {
        const std::set<double> mandatory_sample_set = get_road_mandatory_s_samples(road, refinement_start, refinement_end);
        samples.insert(mandatory_sample_set.begin(), mandatory_sample_set.end());
    }

    const std::vector<double> mandatory_samples(samples.begin(), samples.end());
    for (std::size_t idx = 1; idx < mandatory_samples.size(); idx++)
    {
        const double interval_start = mandatory_samples[idx - 1];
        const double interval_end = mandatory_samples[idx];
        const double interval_end_t = interval_end == s_end ? t_end : get_t(interval_end);
        if (interval_start >= 0.0 && interval_end <= road.length)
            refine_interval(samples, interval_start, get_t(interval_start), interval_end, interval_end_t, eps);
    }
    return samples;
}

void RoadSampler::refine_interval(std::set<double>& samples, double s_start, double t_start, double s_end, double t_end, double eps) const
{
    CubicBounds lateral;
    lateral.min = std::min(t_start, t_end);
    lateral.max = std::max(t_start, t_end);
    lateral.d1 = std::abs((t_end - t_start) / (s_end - s_start));

    const IntervalBounds interval = get_road_interval_bounds(road, s_start, s_end);
    const double         error = curve_error(interval, lateral, DerivativeBounds{});
    if (std::isfinite(error) && std::nextafter(error, std::numeric_limits<double>::infinity()) <= eps)
    {
        samples.insert(s_end);
        return;
    }

    const double s_mid = 0.5 * (s_start + s_end);
    const double t_mid = 0.5 * (t_start + t_end);
    require_or_throw(s_mid > s_start && s_mid < s_end, "eps {} is too small to represent road line", eps);
    refine_interval(samples, s_start, t_start, s_mid, t_mid, eps);
    refine_interval(samples, s_mid, t_mid, s_end, t_end, eps);
}

LaneSampler::LaneSampler(const Lane& lane) :
    lane(lane),
    lane_section(*get_parent_or_throw<LaneSection>(lane)),
    road(*get_parent_or_throw<Road>(lane_section)),
    inner_lane(lane_section.id_to_lane.at(next_towards_zero(lane.id)))
{
}

EdgePoints LaneSampler::edge_points(double s) const
{
    const double outer_t = lane.outer_border.evaluate(s).value_or(0.0);
    const double inner_t = inner_lane.outer_border.evaluate(s).value_or(0.0);
    return {lane.get_surface_pt(s, inner_t), lane.get_surface_pt(s, outer_t)};
}

std::set<double> LaneSampler::get_border_s_samples(double s_start, double s_end, double t_offset, double eps, bool allow_extrapolate) const
{
    require_or_throw(std::isfinite(eps) && eps > 0, "eps must be finite and greater than 0 (got {})", eps);
    require_or_throw(std::isfinite(t_offset), "t offset must be finite (got {})", t_offset);
    require_or_throw(std::isfinite(s_start) && std::isfinite(s_end) && s_start < s_end, "invalid lane border range [{}, {}]", s_start, s_end);

    const double lane_section_end = lane_section.get_end();
    require_or_throw(allow_extrapolate || (s_start >= lane_section.s && s_end <= lane_section_end),
                     "lane border range [{}, {}] must be within lane section range [{}, {}]",
                     s_start,
                     s_end,
                     lane_section.s,
                     lane_section_end);

    // only refine for s-range of lane section, linear extrapolate past ends
    const double refinement_start = std::max(s_start, lane_section.s);
    const double refinement_end = std::min(s_end, lane_section_end);

    std::set<double> samples{s_start, s_end};
    if (refinement_start >= refinement_end)
        return samples;

    const std::set<double>    mandatory_sample_set = get_lane_mandatory_s_samples(road, lane, inner_lane, refinement_start, refinement_end);
    const std::vector<double> mandatory_samples(mandatory_sample_set.begin(), mandatory_sample_set.end());
    samples.insert(mandatory_sample_set.begin(), mandatory_sample_set.end());

    for (std::size_t idx = 1; idx < mandatory_samples.size(); idx++)
        refine_border_interval(samples, mandatory_samples[idx - 1], mandatory_samples[idx], t_offset, eps);

    return samples;
}

std::set<double> LaneSampler::get_mesh_s_samples(double s_start, double s_end, double eps, bool allow_extrapolate) const
{
    require_or_throw(std::isfinite(eps) && eps > 0, "eps must be finite and greater than 0 (got {})", eps);
    require_or_throw(std::isfinite(s_start) && std::isfinite(s_end) && s_start < s_end, "invalid lane mesh range [{}, {}]", s_start, s_end);

    const double lane_section_end = lane_section.get_end();
    require_or_throw(allow_extrapolate || (s_start >= lane_section.s && s_end <= lane_section_end),
                     "lane mesh range [{}, {}] must be within lane section range [{}, {}]",
                     s_start,
                     s_end,
                     lane_section.s,
                     lane_section_end);

    // only refine for s-range of lane section, linear extrapolate past ends
    const double refinement_start = std::max(s_start, lane_section.s);
    const double refinement_end = std::min(s_end, lane_section_end);

    std::set<double> samples{s_start, s_end};
    if (refinement_start >= refinement_end)
        return samples;

    const std::set<double> mandatory_sample_set = get_lane_mandatory_s_samples(road, lane, inner_lane, refinement_start, refinement_end);
    samples.insert(mandatory_sample_set.begin(), mandatory_sample_set.end());

    // sorted and unique; not a std::set for vector-like access
    const std::vector<double> mandatory_samples(mandatory_sample_set.begin(), mandatory_sample_set.end());

    EdgePoints start = edge_points(mandatory_samples.front());
    for (std::size_t idx = 1; idx < mandatory_samples.size(); idx++)
    {
        const EdgePoints end = edge_points(mandatory_samples[idx]);
        refine_interval(samples, mandatory_samples[idx - 1], mandatory_samples[idx], start, end, eps);
        start = end;
    }

    return samples;
}

void LaneSampler::refine_border_interval(std::set<double>& samples, double s_start, double s_end, double t_offset, double eps) const
{
    const double         s_mid = 0.5 * (s_start + s_end);
    const IntervalBounds interval = get_lane_interval_bounds(road, lane, inner_lane, s_start, s_end);

    CubicBounds border_line = interval.outer;
    border_line.min += t_offset;
    border_line.max += t_offset;

    DerivativeBounds height;
    if (lane.level.value_or(false))
    {
        height = product(interval.crossfall, magnitudes(interval.inner, true));
        CubicBounds inner_offset = interval.width;
        inner_offset.min += t_offset;
        inner_offset.max += t_offset;
        height = sum(height, product(tan_bounds(interval.theta), magnitudes(inner_offset, false)));
    }
    else
        height = product(interval.crossfall, magnitudes(border_line, true));

    // Lane heights are linearly interpolated between the inner and outer lane
    // borders. At outer_border + t_offset this is
    // outer_height + t_offset * (outer_height - inner_height) / lane_width.
    const double height_delta = interval.outer_height_offset - interval.inner_height_offset;
    if (height_delta == 0)
        height.value += std::abs(interval.inner_height_offset);
    else if (interval.width.min == 0 && interval.width.max == 0)
        height.value += std::abs(interval.inner_height_offset); // get_surface_pt uses the inner height for a zero-width lane
    else
    {
        if (t_offset == 0)
            height.value += std::abs(interval.outer_height_offset);
        else if (interval.width.min <= 0 && interval.width.max >= 0) // width contains zero -> infinite bounds
        {
            height.value = std::numeric_limits<double>::infinity();
            height.d1 = std::numeric_limits<double>::infinity();
            height.d2 = std::numeric_limits<double>::infinity();
        }
        else
        {
            const double min_abs_width = std::min(std::abs(interval.width.min), std::abs(interval.width.max));
            const double scale = std::abs(t_offset * height_delta);
            height.value += std::abs(interval.outer_height_offset) + scale / min_abs_width;
            height.d1 += scale * interval.width.d1 / (min_abs_width * min_abs_width);
            height.d2 += scale * (interval.width.d2 / (min_abs_width * min_abs_width) +
                                  2 * interval.width.d1 * interval.width.d1 / (min_abs_width * min_abs_width * min_abs_width));
        }
    }

    const double error = curve_error(interval, border_line, height);
    if (std::isfinite(error) && std::nextafter(error, std::numeric_limits<double>::infinity()) <= eps)
        return;

    require_or_throw(s_mid > s_start && s_mid < s_end, "eps {} is too small to represent lane border", eps);
    samples.insert(s_mid);
    refine_border_interval(samples, s_start, s_mid, t_offset, eps);
    refine_border_interval(samples, s_mid, s_end, t_offset, eps);
}

void LaneSampler::refine_interval(
    std::set<double>& samples, double s_start, double s_end, const EdgePoints& start, const EdgePoints& end, double eps) const
{
    const double         s_mid = 0.5 * (s_start + s_end);
    const IntervalBounds interval = get_lane_interval_bounds(road, lane, inner_lane, s_start, s_end);

    const auto edge_error = [&](const CubicBounds& edge, bool outer, double height_offset)
    {
        DerivativeBounds height = product(interval.crossfall, magnitudes(interval.inner, true));
        if (!lane.level.value_or(false))
            height = product(interval.crossfall, magnitudes(edge, true));
        else if (outer)
        {
            CubicBounds width;
            width.min = edge.min - interval.inner.max;
            width.max = edge.max - interval.inner.min;
            width.d1 = edge.d1 + interval.inner.d1;
            width.d2 = edge.d2 + interval.inner.d2;
            width.d3 = edge.d3 + interval.inner.d3;
            height = sum(height, product(tan_bounds(interval.theta), magnitudes(width, false)));
        }
        height.value += std::abs(height_offset);

        return curve_error(interval, edge, height);
    };

    // splitting a bilinear ruled patch into two triangles adds at most one quarter of its twist vector
    const double twist = 0.25 * norm(add(sub(start.outer, end.outer), sub(end.inner, start.inner)));
    const double error =
        std::max(edge_error(interval.outer, true, interval.outer_height_offset), edge_error(interval.inner, false, interval.inner_height_offset)) +
        twist;
    if (std::isfinite(error) && std::nextafter(error, std::numeric_limits<double>::infinity()) <= eps)
        return;

    require_or_throw(s_mid > s_start && s_mid < s_end, "eps {} is too small to represent lane mesh", eps);
    samples.insert(s_mid);
    const EdgePoints mid = edge_points(s_mid);
    refine_interval(samples, s_start, s_mid, start, mid, eps);
    refine_interval(samples, s_mid, s_end, mid, end, eps);
}

} // namespace odr
