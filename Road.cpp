#include "Road.h"
#include "RefLine.h"

#include <array>
#include <cmath>
#include <iterator>
#include <math.h>
#include <utility>

namespace odr
{
double Crossfall::get_crossfall(double s, bool on_left_side) const
{
    const Poly3 poly = this->get_poly(s);

    Side side = Side::Both; // applicable side of the road
    if (this->sides.find(poly.s_start) != this->sides.end())
        side = this->sides.at(poly.s_start);

    if (on_left_side && side == Side::Right)
        return 0;
    else if (!on_left_side && side == Side::Left)
        return 0;

    return poly.get(s);
}

ConstLaneSectionSet Road::get_lanesections() const
{
    ConstLaneSectionSet lanesections;
    for (const auto& s0_lansection : this->s0_to_lanesection)
        lanesections.insert(s0_lansection.second);

    return lanesections;
}

LaneSectionSet Road::get_lanesections()
{
    LaneSectionSet lanesections;
    for (const auto& s0_lansection : this->s0_to_lanesection)
        lanesections.insert(s0_lansection.second);

    return lanesections;
}

std::shared_ptr<const LaneSection> Road::get_lanesection(double s) const
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

std::shared_ptr<LaneSection> Road::get_lanesection(double s)
{
    std::shared_ptr<LaneSection> lanesection = std::const_pointer_cast<LaneSection>(static_cast<const Road&>(*this).get_lanesection(s));
    return lanesection;
}

std::shared_ptr<const Lane> Road::get_lane(double s, double t, double* t_outer_brdr) const
{
    std::shared_ptr<const LaneSection> lanesection = this->get_lanesection(s);
    if (!lanesection)
    {
        printf("road #%s - could not get lane section for s %.2f\n", this->id.c_str(), s);
        return nullptr;
    }

    return lanesection->get_lane(s, t, t_outer_brdr);
}

std::shared_ptr<Lane> Road::get_lane(double s, double t, double* t_outer_brdr)
{
    std::shared_ptr<Lane> lane = std::const_pointer_cast<Lane>(static_cast<const Road&>(*this).get_lane(s, t, t_outer_brdr));
    return lane;
}

Vec3D Road::get_xyz(double s, double t, double z, bool with_superelevation) const
{
    const Mat3D trans_mat = this->get_transformation_matrix(s, with_superelevation);
    const Vec3D xyz = MatVecMultiplication(trans_mat, Vec3D{t, z, 1});

    return xyz;
}

Vec3D Road::get_surface_pt(double s, double t) const
{
    std::shared_ptr<const LaneSection> lanesection = this->get_lanesection(s);
    if (!lanesection)
    {
        printf("road #%s - could not get lane section for s: %.2f\n", this->id.c_str(), s);
        return this->get_xyz(s, t, 0.0);
    }

    double                      t_outer_brdr = 0;
    std::shared_ptr<const Lane> lane = this->get_lane(s, t, &t_outer_brdr);
    if (!lane)
    {
        printf("road #%s - could not get lane for s: %.2f t: %.2f\n", this->id.c_str(), s, t);
        return this->get_xyz(s, t, 0.0);
    }

    if (lane->id == 0)
        return this->get_xyz(s, t, 0.0);

    double z_offs = 0;
    if (lane->level)
    {
        const double lane_width = lane->lane_width.get(s - lanesection->s0);
        const double t_inner_brdr = (lane->id > 0) ? t_outer_brdr - lane_width : t_outer_brdr + lane_width;
        const double superelev = this->superelevation.get(s); // cancel out superelevation
        const double h_inner_brdr = -std::tan(this->crossfall.get_crossfall(s, (lane->id > 0))) * std::abs(t_inner_brdr);
        z_offs = h_inner_brdr + std::tan(superelev) * (t - t_inner_brdr);
    }
    else
    {
        z_offs = -std::tan(this->crossfall.get_crossfall(s, (lane->id > 0))) * std::abs(t);
    }

    return this->get_xyz(s, t, z_offs);
}

Mat3D Road::get_transformation_matrix(double s, bool with_superelevation) const
{
    const Vec3D  s_vec = this->ref_line->get_grad(s);
    const double superelevation = with_superelevation ? this->superelevation.get(s) : 0;

    const Vec3D e_t = normalize(Vec3D{-s_vec[1], s_vec[0], std::tan(superelevation) * std::abs(s_vec[1])});
    const Vec3D e_z = normalize(crossProduct(s_vec, e_t));
    const Vec3D p0 = this->ref_line->get_xyz(s);

    const Mat3D trans_mat{{{e_t[0], e_z[0], p0[0]}, {e_t[1], e_z[1], p0[1]}, {e_t[2], e_z[2], p0[2]}}};

    return trans_mat;
}

} // namespace odr