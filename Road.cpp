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

    if (this->s_start_to_poly.size() > 0)
    {
        auto target_poly_iter = this->s_start_to_poly.upper_bound(s);
        if (target_poly_iter != this->s_start_to_poly.begin())
            target_poly_iter--;

        Side side = Side::Both; // applicable side of the road
        if (this->sides.find(target_poly_iter->first) != this->sides.end())
            side = this->sides.at(target_poly_iter->first);

        if (on_left_side && side == Side::Right)
            return 0;
        else if (!on_left_side && side == Side::Left)
            return 0;

        return target_poly_iter->second.get(s);
    }

    return 0;
}

ConstLaneSectionSet Road::get_lanesections() const
{
    ConstLaneSectionSet lanesections;
    for (const auto& s_lansection : this->s_to_lanesection)
        lanesections.insert(s_lansection.second);

    return lanesections;
}

LaneSectionSet Road::get_lanesections()
{
    LaneSectionSet lanesections;
    for (const auto& s_lansection : this->s_to_lanesection)
        lanesections.insert(s_lansection.second);

    return lanesections;
}

std::shared_ptr<const LaneSection> Road::get_lanesection(double s) const
{
    if (this->s_to_lanesection.size() > 0)
    {
        auto target_lane_sec_iter = this->s_to_lanesection.upper_bound(s);
        if (target_lane_sec_iter != this->s_to_lanesection.begin())
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

Vec3D Road::get_xyz(double s, double t, double z, bool with_superelevation) const
{
    const Mat3D trans_mat = this->get_transformation_matrix(s, with_superelevation);
    const Vec3D xyz = MatVecMultiplication(trans_mat, Vec3D{t, z, 1});

    return xyz;
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

Vec3D Road::get_surface_pt(double s, double t, bool with_lateralProfile, bool with_laneHeight) const
{
    std::shared_ptr<const LaneSection> lanesection = this->get_lanesection(s);
    if (!lanesection)
    {
        printf("road #%s - could not get lane section for s: %.2f\n", this->id.c_str(), s);
        return this->get_xyz(s, t, 0.0);
    }

    std::shared_ptr<const Lane> lane = lanesection->get_lane(s, t);

    const double t_inner_brdr = lane->inner_border.get(s);
    double       z_t = 0;

    if (with_lateralProfile)
    {
        const double z_inner_brdr = -std::tan(this->crossfall.get_crossfall(s, (lane->id > 0))) * std::abs(t_inner_brdr);
        if (lane->level)
        {
            const double superelev = this->superelevation.get(s); // cancel out superelevation
            z_t = z_inner_brdr + std::tan(superelev) * (t - t_inner_brdr);
        }
        else
        {
            z_t = -std::tan(this->crossfall.get_crossfall(s, (lane->id > 0))) * std::abs(t);
        }
    }

    if (with_laneHeight && lane->s_to_height_offset.size() > 0)
    {
        const std::map<double, HeightOffset>& height_offs = lane->s_to_height_offset;

        auto s0_height_offs_iter = height_offs.upper_bound(s);
        if (s0_height_offs_iter != height_offs.begin())
            s0_height_offs_iter--;

        const double t_outer_brdr = lane->outer_border.get(s);
        const double inner_height = s0_height_offs_iter->second.inner;
        const double outer_height = s0_height_offs_iter->second.outer;
        const double p_t = (t - t_inner_brdr) / (t_outer_brdr - t_inner_brdr);
        z_t += p_t * (outer_height - inner_height) + inner_height;

        if (std::next(s0_height_offs_iter) != height_offs.end())
        {
            /* if successive lane height entry available linearly interpolate */
            const double ds = std::next(s0_height_offs_iter)->first - s0_height_offs_iter->first;
            const double dh_inner = std::next(s0_height_offs_iter)->second.inner - inner_height;
            const double dz_inner = (dh_inner / ds) * (s - lanesection->s0 - s0_height_offs_iter->first);
            const double dh_outer = std::next(s0_height_offs_iter)->second.outer - outer_height;
            const double dz_outer = (dh_outer / ds) * (s - lanesection->s0 - s0_height_offs_iter->first);

            z_t += p_t * (dz_outer - dz_inner) + dz_inner;
        }
    }

    return this->get_xyz(s, t, z_t, with_lateralProfile);
}

} // namespace odr