#include "Road.h"

#include <cstring>
#include <iostream>

#include "Lanes.h"

namespace odr
{

Road::Road(double length, int id, int junction)
    : id(id), junction(junction), length(length) {}

Vec3D Road::get_xyz(double s, double t, double z) const
{
    const Mat3D trans_mat = this->get_transformation_matrix(s);
    const Vec3D xyz = MatVecMultiplication(trans_mat, Vec3D{1, t, z});
    return xyz;
}

Mat3D Road::get_transformation_matrix(double s) const
{
    const Vec3D s_vec = this->ref_line->get_grad(s);
    const Vec3D e_t = normalize(Vec3D{-s_vec[1], s_vec[0], 0});
    const Vec3D e_z = normalize(crossProduct(s_vec, e_t));

    const Vec3D p0 = this->ref_line->get_xyz(s);
    const Mat3D trans_mat{{{p0[0], e_t[0], e_z[0]},
                           {p0[1], e_t[1], e_z[1]},
                           {p0[2], e_t[2], e_z[2]}}};

    return trans_mat;
}

double Road::get_lane_offset(double s) const
{
    double offs = 0;
    if (this->lane_offsets.size() > 0)
    {
        auto target_lane_offset_iter = this->lane_offsets.upper_bound(s);
        if (target_lane_offset_iter != this->lane_offsets.begin())
            target_lane_offset_iter--;
        offs = target_lane_offset_iter->second->get_offset(s);
    }
    return offs;
}

} // namespace odr