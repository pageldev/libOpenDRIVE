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

    Vec3D       e_s, e_t, e_h;
    const Vec3D p0 = road_ptr->get_xyz(this->s0, this->t0, this->z0, &e_s, &e_t, &e_h);

    const Mat3D rot_mat = EulerAnglesToMatrix<double>(roll, pitch, hdg);
    const Vec3D e_u = MatVecMultiplication(rot_mat, e_s);
    const Vec3D e_v = MatVecMultiplication(rot_mat, e_t);
    const Vec3D e_z = MatVecMultiplication(rot_mat, e_h);

    Mesh3D road_obj_mesh;

    if (radius > 0)
    {
        road_obj_mesh.vertices.push_back(p0);
        const double eps_angle = std::acos((radius * radius - 4 * radius * eps + 2 * eps * eps) / (radius * radius));
        for (double alpha = 0; alpha < 2 * M_PI; alpha += eps_angle)
        {
            const Vec3D circle_pt = {p0[0] + radius * std::cos(alpha), p0[1] + radius * std::sin(alpha), p0[2]};
            road_obj_mesh.vertices.push_back(circle_pt);
            if (road_obj_mesh.vertices.size() > 2)
            {
                const size_t          cur_idx = road_obj_mesh.vertices.size() - 1;
                std::array<size_t, 3> idx_patch = {0, cur_idx - 1, cur_idx};
                road_obj_mesh.indices.insert(road_obj_mesh.indices.end(), idx_patch.begin(), idx_patch.end());
            }
        }
    }

    return road_obj_mesh;
}
} // namespace odr