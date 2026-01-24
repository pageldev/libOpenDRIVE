#include "RoadSignal.h"
#include "Math.hpp"
#include "Utils.hpp"

#include <algorithm>
#include <cstdint>

namespace odr
{

RoadSignal::RoadSignal(std::string                id,
                       std::string                road_id,
                       double                     s0,
                       double                     t0,
                       double                     zOffset,
                       bool                       is_dynamic,
                       std::string                type,
                       std::string                subtype,
                       std::string                orientation,
                       std::optional<double>      value,
                       std::optional<double>      height,
                       std::optional<double>      width,
                       std::optional<double>      hOffset,
                       std::optional<double>      pitch,
                       std::optional<double>      roll,
                       std::optional<std::string> name,
                       std::optional<std::string> unit,
                       std::optional<std::string> text,
                       std::optional<std::string> country) :
    id(id),
    road_id(road_id),
    s0(s0),
    t0(t0),
    zOffset(zOffset),
    is_dynamic(is_dynamic),
    type(type),
    subtype(subtype),
    orientation(orientation),
    value(value),
    height(height),
    width(width),
    hOffset(hOffset),
    pitch(pitch),
    roll(roll),
    name(name),
    unit(unit),
    text(text),
    country(country)
{
    require_or_throw(s0 >= 0, "s {} < 0", s0);
    require_or_throw(!std::isnan(t0), "t is NaN");
    require_or_throw(!std::isnan(zOffset), "zOffset is NaN");
    require_or_throw(!height || *height >= 0, "height < 0");
    require_or_throw(!width || *width >= 0, "width < 0");
}

Mesh3D RoadSignal::get_box(double w, double l, double h)
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
