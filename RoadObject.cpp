#include "RoadObject.h"
#include "Road.h"

#include <math.h>

namespace odr
{
Mesh3D RoadObject::get_cylinder(double eps, double radius, double height)
{
    Mesh3D cylinder_mesh;
    cylinder_mesh.vertices.push_back({0, 0, 0});
    cylinder_mesh.vertices.push_back({0, 0, height});

    eps = 0.5 * eps; // reduce eps a bit, cylinders more subsceptible to low resolution
    const double eps_angle = (radius <= eps) ? M_PI / 6 : std::acos((radius * radius - 4 * radius * eps + 2 * eps * eps) / (radius * radius));

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
            const size_t          cur_idx = cylinder_mesh.vertices.size() - 1;
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

Mesh3D RoadObject::get_mesh(double eps) const
{
    auto road_ptr = this->road.lock();
    if (!road_ptr)
        throw std::runtime_error("could not access parent road for road object");

    std::vector<RoadObjectRepeat> repeats_copy = this->repeats;
    if (repeats_copy.empty())
        repeats_copy.push_back({s0, 0, 1, t0, t0, width, width, height, height, z0, z0});

    const Mat3D rot_mat = EulerAnglesToMatrix<double>(roll, pitch, hdg);

    Mesh3D road_obj_mesh;
    for (const RoadObjectRepeat& repeat : repeats_copy)
    {
        const double s_start = repeat.s0;
        const double s_end = repeat.s0 + std::max(repeat.length, 1e-12); // avoid division by zero

        if (repeat.distance != 0)
        {
            for (double s = s_start; (s - s_end) < 1e-9 && s < road_ptr->length; s += repeat.distance)
            {
                const double progress = (s - s_start) / (s_end - s_start);
                const double t_s = repeat.t_start + progress * (repeat.t_end - repeat.t_start);
                const double z_s = repeat.z_offset_start + progress * (repeat.z_offset_end - repeat.z_offset_start);
                const double height_s = repeat.height_start + progress * (repeat.height_end - repeat.height_start);
                const double width_s = repeat.width_start + progress * (repeat.width_end - repeat.width_start);

                Mesh3D single_road_obj_mesh;
                if (this->radius > 0)
                {
                    single_road_obj_mesh = this->get_cylinder(eps, this->radius, height_s);
                }
                else if ((this->width > 0 || width_s > 0) && (this->length > 0))
                {
                    const double w = (width_s > 0) ? width_s : this->width;
                    const double h = (height_s > 0) ? height_s : this->height;
                    single_road_obj_mesh = this->get_box(w, this->length, h);
                }

                /* apply rotation and transform s/t/h -> x/y/z */
                for (Vec3D& pt_uvz : single_road_obj_mesh.vertices)
                {
                    pt_uvz = MatVecMultiplication(rot_mat, pt_uvz);
                    pt_uvz = add(pt_uvz, {s, t_s, z_s});
                    pt_uvz = road_ptr->get_xyz(std::min(pt_uvz[0], road_ptr->length), pt_uvz[1], pt_uvz[2]);
                }

                road_obj_mesh.add_mesh(single_road_obj_mesh);

                if (repeat.distance < 0)
                    break;
            }
        }
    }

    return road_obj_mesh;
}
} // namespace odr