#include "RoadSignal.h"

namespace odr
{
RoadSignal::RoadSignal(std::string road_id,
                       std::string id,
                       std::string name,
                       double      s0,
                       double      t0,
                       bool        is_dynamic,
                       double      zOffset,
                       double      value,
                       double      height,
                       double      width,
                       double      hOffset,
                       double      pitch,
                       double      roll,
                       std::string orientation,
                       std::string country,
                       std::string type,
                       std::string subtype,
                       std::string unit,
                       std::string text) :
    road_id(road_id),
    id(id), name(name), s0(s0), t0(t0), is_dynamic(is_dynamic), zOffset(zOffset), value(value), height(height), width(width), hOffset(hOffset),
    pitch(pitch), roll(roll), orientation(orientation), country(country), type(type), subtype(subtype), unit(unit), text(text)
{
}

Mesh3D RoadSignal::get_box(const double w, const double l, const double h)
{
    Mesh3D box_mesh;
    box_mesh.vertices = {Vec3D{l / 2, w / 2, 0},
                         Vec3D{-l / 2, w / 2, 0},
                         Vec3D{-l / 2, -w / 2, 0},
                         Vec3D{l / 2, -w / 2, 0},
                         Vec3D{l / 2, w / 2, h},
                         Vec3D{-l / 2, w / 2, h},
                         Vec3D{-l / 2, -w / 2, h},
                         Vec3D{l / 2, -w / 2, h}};
    box_mesh.indices = {0, 3, 1, 3, 2, 1, 4, 5, 7, 7, 5, 6, 7, 6, 3, 3, 6, 2, 5, 4, 1, 1, 4, 0, 0, 4, 7, 7, 3, 0, 1, 6, 5, 1, 2, 6};

    return box_mesh;
}
} // namespace odr
