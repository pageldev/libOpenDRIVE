#include "RoadSignal.h"

namespace odr {

    RoadSignal::RoadSignal(std::string road_id,
                           std::string id,
                           double s0,
                           double t0,
                           double z0,
                           double width,
                           double height,
                           double roll,
                           double pitch,
                           double yaw,
                           double h_offset,
                           double value,
                           std::string country,
                           std::string country_revision,
                           std::string type,
                           std::string subtype,
                           std::string text,
                           std::string unit,
                           std::string name,
                           bool dynamic):
                           road_id(road_id),
                           id(id), s0(s0), t0(t0),z0(z0),width(width), height(height), roll(roll), pitch(pitch),yaw(yaw),
                           hdg(h_offset),name(name) ,value(value), country(country), country_revision(country_revision),
                           type(type), subtype(subtype), text(text), unit(unit),dynamic(dynamic)
                           {}
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

}// namespace odr