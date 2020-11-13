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
    const Vec3D xyz = MatVecMultiplication(trans_mat, Vec3D{t, z, 1});
    return xyz;
}

Mat3D Road::get_transformation_matrix(double s) const
{
    const Vec3D  s_vec = this->ref_line->get_grad(s);
    const double superelevation = this->superelevation.get(s);

    const Vec3D e_t = normalize(Vec3D{-s_vec[1], s_vec[0], std::tan(superelevation) / s_vec[0]});
    const Vec3D e_z = normalize(crossProduct(s_vec, e_t));
    const Vec3D p0 = this->ref_line->get_xyz(s);

    const Mat3D trans_mat{{{e_t[0], e_z[0], p0[0]},
                           {e_t[1], e_z[1], p0[1]},
                           {e_t[2], e_z[2], p0[2]}}};

    return trans_mat;
}

} // namespace odr