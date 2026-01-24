#include "RoadObject.h"
#include "Utils.hpp"

#include "fmt/format.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace odr
{

RoadObjectRepeat::RoadObjectRepeat(double                s0,
                                   double                length,
                                   double                distance,
                                   double                t_start,
                                   double                t_end,
                                   double                height_start,
                                   double                height_end,
                                   double                z_offset_start,
                                   double                z_offset_end,
                                   std::optional<double> width_start,
                                   std::optional<double> width_end) :
    s0(s0),
    length(length),
    distance(distance),
    t_start(t_start),
    t_end(t_end),
    height_start(height_start),
    height_end(height_end),
    z_offset_start(z_offset_start),
    z_offset_end(z_offset_end),
    width_start(width_start),
    width_end(width_end)
{
    require_or_throw(s0 >= 0, "s {} < 0", s0);
    require_or_throw(length >= 0, "length {} < 0", length);
    require_or_throw(distance >= 0, "distance {} < 0", distance);
    require_or_throw(!std::isnan(t_start), "tStart is NaN");
    require_or_throw(!std::isnan(t_end), "tEnd is NaN");
    require_or_throw(height_start >= 0, "heightStart {} < 0", height_start);
    require_or_throw(height_end >= 0, "heightEnd {} < 0", height_end);
    require_or_throw(!std::isnan(z_offset_start), "zOffsetStart is NaN");
    require_or_throw(!std::isnan(z_offset_end), "zOffsetEnd is NaN");
    require_or_throw(!width_start || *width_start >= 0, "widthStart < 0");
    require_or_throw(!width_end || *width_end >= 0, "widthEnd < 0");
}

RoadObjectCorner::RoadObjectCorner(Vec3D pt, double height, Type type, std::optional<int> id) : pt(pt), height(height), type(type), id(id)
{
    require_or_throw(std::none_of(pt.begin(), pt.end(), [](double v) { return std::isnan(v); }), "pt [{}] has NaN values", fmt::join(pt, ", "));
    require_or_throw(height >= 0, "height {} < 0", height);
    if (type == Type::Road)
        require_or_throw(pt[0] >= 0, "s {} < 0", pt[0]);
}

RoadObjectOutline::RoadObjectOutline(std::optional<int>         id,
                                     std::optional<std::string> fill_type,
                                     std::optional<std::string> lane_type,
                                     std::optional<bool>        outer,
                                     std::optional<bool>        closed) :
    id(id), fill_type(fill_type), lane_type(lane_type), outer(outer), closed(closed)
{
}

RoadObject::RoadObject(std::string                road_id,
                       std::string                id,
                       double                     s0,
                       double                     t0,
                       double                     z0,
                       std::optional<double>      length,
                       std::optional<double>      valid_length,
                       std::optional<double>      width,
                       std::optional<double>      radius,
                       std::optional<double>      height,
                       std::optional<double>      hdg,
                       std::optional<double>      pitch,
                       std::optional<double>      roll,
                       std::optional<std::string> type,
                       std::optional<std::string> name,
                       std::optional<std::string> orientation,
                       std::optional<std::string> subtype,
                       std::optional<bool>        is_dynamic) :
    road_id(road_id),
    id(id),
    s0(s0),
    t0(t0),
    z0(z0),
    length(length),
    valid_length(valid_length),
    width(width),
    radius(radius),
    height(height),
    hdg(hdg),
    pitch(pitch),
    roll(roll),
    type(type),
    name(name),
    orientation(orientation),
    subtype(subtype),
    is_dynamic(is_dynamic)
{
    require_or_throw(s0 >= 0, "s {} < 0", s0);
    require_or_throw(!std::isnan(t0), "t is NaN");
    require_or_throw(!std::isnan(z0), "z is NaN");
    require_or_throw(!length || *length > 0, "length <= 0");
    require_or_throw(!valid_length || *valid_length >= 0, "validLength < 0");
    require_or_throw(!width || !std::isnan(*width), "width is NaN");
    require_or_throw(!radius || *radius > 0, "radius <= 0");
    require_or_throw(!height || *height >= 0, "height < 0");
    require_or_throw(!hdg || !std::isnan(*hdg), "hdg is NaN");
    require_or_throw(!pitch || !std::isnan(*pitch), "pitch is NaN");
    require_or_throw(!roll || !std::isnan(*roll), "roll is NaN");
}

Mesh3D RoadObject::get_cylinder(double eps, double radius, double height)
{
    Mesh3D cylinder_mesh;
    cylinder_mesh.vertices.push_back({0, 0, 0});
    cylinder_mesh.vertices.push_back({0, 0, height});

    const double eps_adj = 0.5 * eps; // reduce eps a bit, cylinders more subsceptible to low resolution
    const double eps_angle =
        (radius <= eps_adj) ? M_PI / 6 : std::acos((radius * radius - 4 * radius * eps_adj + 2 * eps_adj * eps_adj) / (radius * radius));

    std::vector<double> angles;
    for (double alpha = 0; alpha < 2 * M_PI; alpha += eps_angle)
        angles.push_back(alpha);
    angles.push_back(2 * M_PI);

    for (const double& alpha : angles)
    {
        const Vec3D circle_pt_bottom = {radius * std::cos(alpha), radius * std::sin(alpha), 0};
        const Vec3D circle_pt_top = {radius * std::cos(alpha), radius * std::sin(alpha), height};
        cylinder_mesh.vertices.push_back(circle_pt_bottom);
        cylinder_mesh.vertices.push_back(circle_pt_top);

        if (cylinder_mesh.vertices.size() > 5)
        {
            const std::size_t     cur_idx = cylinder_mesh.vertices.size() - 1;
            std::array<size_t, 6> top_bottom_idx_patch = {0, cur_idx - 1, cur_idx - 3, 1, cur_idx - 2, cur_idx};
            cylinder_mesh.indices.insert(cylinder_mesh.indices.end(), top_bottom_idx_patch.begin(), top_bottom_idx_patch.end());
            std::array<size_t, 6> wall_idx_patch = {cur_idx, cur_idx - 2, cur_idx - 3, cur_idx, cur_idx - 3, cur_idx - 1};
            cylinder_mesh.indices.insert(cylinder_mesh.indices.end(), wall_idx_patch.begin(), wall_idx_patch.end());
        }
    }

    return cylinder_mesh;
}

Mesh3D RoadObject::get_box(double w, double l, double h)
{
    return Mesh3D({Vec3D{l / 2, w / 2, 0},
                   Vec3D{-l / 2, w / 2, 0},
                   Vec3D{-l / 2, -w / 2, 0},
                   Vec3D{l / 2, -w / 2, 0},
                   Vec3D{l / 2, w / 2, h},
                   Vec3D{-l / 2, w / 2, h},
                   Vec3D{-l / 2, -w / 2, h},
                   Vec3D{l / 2, -w / 2, h}},
                  {0, 3, 1, 3, 2, 1, 4, 5, 7, 7, 5, 6, 7, 6, 3, 3, 6, 2, 5, 4, 1, 1, 4, 0, 0, 4, 7, 7, 3, 0, 1, 6, 5, 1, 2, 6},
                  {},
                  {});
}

} // namespace odr
