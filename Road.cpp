#include "Road.h"
#include "RefLine.h"

#include <array>
#include <math.h>
#include <utility>

namespace odr
{
double Crossfall::get_crossfall(double s, double t) const
{
    std::shared_ptr<const Poly3> poly = this->get_poly(s);
    if (poly)
    {
        Side side = Side::Both;
        if (this->sides.find(poly->s0) != this->sides.end())
            side = this->sides.at(poly->s0);

        if (t > 0 /*left*/ && side == Side::Right)
            return 0;
        else if (t < 0 /*right*/ && side == Side::Left)
            return 0;

        return poly->get(s) * -static_cast<double>(sign(t));
    }

    return 0;
}

Road::Road(double length, int id, int junction) : id(id), junction(junction), length(length) {}

std::shared_ptr<LaneSection> Road::get_lanesection(double s)
{
    if (this->s0_to_lanesection.size() > 0)
    {
        auto target_lane_sec_iter = this->s0_to_lanesection.upper_bound(s);
        if (target_lane_sec_iter != this->s0_to_lanesection.begin())
            target_lane_sec_iter--;
        return target_lane_sec_iter->second;
    }

    return nullptr;
}

std::shared_ptr<Lane> Road::get_lane(double s, double t)
{
    std::shared_ptr<LaneSection> lanesection = this->get_lanesection(s);
    if (!lanesection)
        return nullptr;
    return lanesection->get_lane(s, t);
}

LaneSectionSet Road::get_lanesections()
{
    LaneSectionSet lanesections;
    for (const auto& s0_lansection : this->s0_to_lanesection)
        lanesections.insert(s0_lansection.second);

    return lanesections;
}

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

    const Mat3D trans_mat{{{e_t[0], e_z[0], p0[0]}, {e_t[1], e_z[1], p0[1]}, {e_t[2], e_z[2], p0[2]}}};

    return trans_mat;
}

} // namespace odr