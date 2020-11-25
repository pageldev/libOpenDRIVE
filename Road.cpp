#include "Road.h"
#include "RefLine.h"

#include <array>
#include <cmath>
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

        return poly->get(s);
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

Vec3D Road::get_surface_pt(double s, double t)
{
    std::shared_ptr<LaneSection> lanesection = this->get_lanesection(s);
    if (!lanesection || t == 0)
        return this->get_xyz(s, t, 0.0);

    std::map<int, double> lane_id_to_outer_brdr = lanesection->get_lane_borders(s);

    /* adjust borders - center lane #0 at t=0 again */
    const double          t_offset = lane_id_to_outer_brdr.at(0);
    std::map<double, int> borders_adj;
    for (const auto& lane_id_brdr : lane_id_to_outer_brdr)
    {
        borders_adj[lane_id_brdr.second - t_offset] = lane_id_brdr.first;
    }

    /* at t lanes in interval [t0, t1] are covered for adjusted borders */
    const double t_adj = t - t_offset;
    const double t_min = borders_adj.begin()->first;
    const double t_max = std::prev(borders_adj.end())->first;
    const double t0 = std::max(std::min(std::min(t_adj, -t_offset), t_max), t_min);
    const double t1 = std::max(std::min(std::max(t_adj, -t_offset), t_max), t_min);

    /* get lane for t0 - can not be .end, but lane #0 */
    auto iter0 = (t0 < 0) ? std::prev(borders_adj.upper_bound(t0)) : borders_adj.lower_bound(t0);

    /* get lane AFTER lane for t1 - can not be .begin, but lane #0 */
    auto iter1 = (t1 < 0) ? borders_adj.upper_bound(t1) : std::next(borders_adj.lower_bound(t1));

    /* get overlaps of individual lanes for interval [t0, t1] */
    std::map<int, double> lane_id_to_overlap;
    if (iter0->second != 0 && iter0 == std::prev(iter1)) // t0 and t1 in same lane
    {
        lane_id_to_overlap[iter0->second] = t1 - t0;
    }
    else
    {
        /* get partial overlap for t0's lane |  t0==>|--->|0|    | */
        if (iter0->second != 0)
        {
            double dt_0 = (t0 < 0) ? std::next(iter0)->first - t0 : iter0->first - t0;
            lane_id_to_overlap[iter0->second] = dt_0;
            iter0++;
        }
        /* get partial overlap for t1's lane |   |0|<---|<==t1 | */
        if (std::prev(iter1)->second != 0)
        {
            double dt_1 = (t1 < 0) ? std::prev(iter1)->first - t1 : t1 - std::prev(iter1, 2)->first;
            lane_id_to_overlap[std::prev(iter1)->second] = dt_1;
            iter1--;
        }
        /* get overlap of fully covered lanes inbetween */
        for (auto it = iter0; it != iter1; it++)
        {
            if (it->second == 0)
                continue;
            double dt = (it->second < 0) ? std::next(it)->first - it->first : it->first - std::prev(it)->first;
            lane_id_to_overlap[it->second] = dt;
        }
    }

    /* get effective overlap - by removing level planes - equals t if all non-level lanes */
    double eff_overlap = 0;
    for (const auto& lane_id_overlap : lane_id_to_overlap)
    {
        const bool is_level = lanesection->id_to_lane.at(lane_id_overlap.first)->level;
        eff_overlap = is_level ? eff_overlap : eff_overlap + lane_id_overlap.second;
    }
    eff_overlap = eff_overlap * static_cast<double>(sign(t));

    /* effectively remove superelevation and don't apply crossfall on level lanes */
    const double superelevation = this->superelevation.get(s);
    const double crossfall = this->crossfall.get_crossfall(s, t);
    const double h = (t - eff_overlap) * std::tan(-superelevation) - std::tan(crossfall) * std::abs(eff_overlap);

    return this->get_xyz(s, t, h);
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