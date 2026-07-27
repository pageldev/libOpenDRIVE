#include "libodr/RoadSignal.h"
#include "libodr/Math.hpp"
#include "libodr/RoadObject.h"
#include "libodr/Utils.hpp"

#include <algorithm>
#include <cstdint>

namespace odr
{

RoadSignal::RoadSignal(const std::string&         id,
                       const std::string&         road_id,
                       double                     s,
                       double                     t,
                       double                     z_offset,
                       bool                       is_dynamic,
                       const std::string&         type,
                       const std::string&         subtype,
                       RoadObject::Orientation    orientation,
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
    s(s),
    t(t),
    z_offset(z_offset),
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
    require_or_throw(s >= 0, "s must be greater than or equal to 0 (got {})", s);
    require_or_throw(!std::isnan(t), "t must not be NaN");
    require_or_throw(!std::isnan(z_offset), "zOffset must not be NaN");
    require_or_throw(!height || height >= 0, "height must be greater than or equal to 0");
    require_or_throw(!width || width >= 0, "width must be greater than or equal to 0");
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
