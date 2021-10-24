#include "RoadObject.h"
#include "Road.h"

#include <math.h>

namespace odr
{
Mesh3D RoadObject::get_mesh(double eps) const
{
    auto road_ptr = this->road.lock();
    if (!road_ptr)
        throw std::runtime_error("could not access parent road for road object");

    /* construct object in local coord system */
    Mesh3D road_obj_mesh;
    if (this->radius > 0)
    {
        /* cylinder */
        road_obj_mesh.vertices.push_back({0, 0, 0});
        road_obj_mesh.vertices.push_back({0, 0, this->height});

        eps = 0.5 * eps; // reduce eps a bit, cylinders more subsceptible to low resolution
        const double eps_angle = (radius <= eps) ? M_PI / 6 : std::acos((radius * radius - 4 * radius * eps + 2 * eps * eps) / (radius * radius));

        std::vector<double> angles;
        for (double alpha = 0; alpha < 2 * M_PI; alpha += eps_angle)
            angles.push_back(alpha);
        angles.push_back(2 * M_PI);

        for (const double& alpha : angles)
        {
            const Vec3D circle_pt_bottom = {radius * std::cos(alpha), radius * std::sin(alpha), 0};
            const Vec3D circle_pt_top = {radius * std::cos(alpha), radius * std::sin(alpha), this->height};
            road_obj_mesh.vertices.push_back(circle_pt_bottom);
            road_obj_mesh.vertices.push_back(circle_pt_top);

            if (road_obj_mesh.vertices.size() > 5)
            {
                const size_t          cur_idx = road_obj_mesh.vertices.size() - 1;
                std::array<size_t, 6> top_bottom_idx_patch = {0, cur_idx - 1, cur_idx - 3, 1, cur_idx - 2, cur_idx};
                road_obj_mesh.indices.insert(road_obj_mesh.indices.end(), top_bottom_idx_patch.begin(), top_bottom_idx_patch.end());
                std::array<size_t, 6> wall_idx_patch = {cur_idx, cur_idx - 2, cur_idx - 3, cur_idx, cur_idx - 3, cur_idx - 1};
                road_obj_mesh.indices.insert(road_obj_mesh.indices.end(), wall_idx_patch.begin(), wall_idx_patch.end());
            }
        }
    }
    else if (this->width > 0 && this->length > 0)
    {
        /* box */
        const double& w = this->width;
        const double& l = this->length;
        const double& h = this->height;

        road_obj_mesh.vertices = {Vec3D{l / 2, w / 2, 0},
                                  Vec3D{-l / 2, w / 2, 0},
                                  Vec3D{-l / 2, -w / 2, 0},
                                  Vec3D{l / 2, -w / 2, 0},
                                  Vec3D{l / 2, w / 2, h},
                                  Vec3D{-l / 2, w / 2, h},
                                  Vec3D{-l / 2, -w / 2, h},
                                  Vec3D{l / 2, -w / 2, h}};
        road_obj_mesh.indices = {0, 3, 1, 3, 2, 1, 4, 5, 7, 7, 5, 6, 7, 6, 3, 3, 6, 2, 5, 4, 1, 1, 4, 0, 0, 4, 7, 7, 3, 0, 1, 6, 5, 1, 2, 6};
    }

    const Mat3D rot_mat = EulerAnglesToMatrix<double>(roll, pitch, hdg);
    for (Vec3D& pt_uvz : road_obj_mesh.vertices)
    {
        pt_uvz = MatVecMultiplication(rot_mat, pt_uvz);
        pt_uvz = add(pt_uvz, {this->s0, this->t0, this->z0});
        pt_uvz = road_ptr->get_xyz(pt_uvz[0], pt_uvz[1], pt_uvz[2]);
    }

    return road_obj_mesh;
}
} // namespace odr