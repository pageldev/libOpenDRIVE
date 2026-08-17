#include "libodr/RoadSignal.h"
#include "libodr/Math.hpp"
#include "libodr/Road.h"
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

Mesh3D RoadSignal::get_mesh(bool enforce_road_bounds) const
{
    const Road& road = *get_parent_or_throw<Road>(*this);

    const Mat3D  rot_mat = EulerAnglesToMatrix<double>(this->roll.value_or(0), this->pitch.value_or(0), this->hOffset.value_or(0));
    const double height = this->height.value_or(DefaultHeight);
    const double width = this->width.value_or(DefaultWidth);

    Mesh3D mesh = get_box(width, Thickness, height);

    Vec3D       e_s, e_t, e_h;
    const Vec3D p0 = road.get_xyz(this->s, this->t, this->z_offset, &e_s, &e_t, &e_h, !enforce_road_bounds);
    const Mat3D base_mat{{{e_s[0], e_t[0], e_h[0]}, {e_s[1], e_t[1], e_h[1]}, {e_s[2], e_t[2], e_h[2]}}};
    for (Vec3D& pt_uvz : mesh.vertices)
    {
        pt_uvz = MatVecMultiplication(rot_mat, pt_uvz);
        pt_uvz = MatVecMultiplication(base_mat, pt_uvz);
        pt_uvz = add(pt_uvz, p0);
        mesh.st_coordinates.push_back({this->s, this->t});
    }

    return mesh;
}
} // namespace odr
