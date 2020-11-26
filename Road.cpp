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

std::shared_ptr<const Lane> Road::get_lane(double s, double t) const
{
    std::map<int, double>              id_to_border = this->get_lane_borders(s);
    std::shared_ptr<const LaneSection> lanesection = this->get_lanesection(s);

    for (auto iter = id_to_border.begin(); iter != id_to_border.end(); iter++)
    {
        const int    lane_id = iter->first;
        const double outer_brdr = iter->second;

        if (lane_id == 0)
        {
            if (id_to_border.at(0) == t)
            {
                return lanesection->id_to_lane.at(0);
            }
            continue;
        }
        else if (lane_id < 0 && t >= outer_brdr && t < std::next(iter)->second)
        {
            return lanesection->id_to_lane.at(lane_id);
        }
        else if (lane_id > 0 && t <= outer_brdr && t > std::prev(iter)->second)
        {
            return lanesection->id_to_lane.at(lane_id);
        }
    }

    return nullptr;
}

std::shared_ptr<Lane> Road::get_lane(double s, double t)
{
    std::shared_ptr<Lane> lane = std::const_pointer_cast<Lane>(static_cast<const Road&>(*this).get_lane(s, t));
    return lane;
}

std::map<int, double> Road::get_lane_borders(double s) const
{
    std::shared_ptr<const LaneSection> lanesection = this->get_lanesection(s);
    if (!lanesection)
        return {};

    auto id_lane_iter0 = lanesection->id_to_lane.find(0);
    if (id_lane_iter0 == lanesection->id_to_lane.end())
        throw std::runtime_error("lane section does not have lane #0");

    std::map<int, double> id_to_outer_border;

    /* iterate from id #0 towards +inf */
    auto id_lane_iter1 = std::next(id_lane_iter0);
    for (auto iter = id_lane_iter1; iter != lanesection->id_to_lane.end(); iter++)
    {
        const double lane_width = iter->second->lane_width.get(s - lanesection->s0);
        id_to_outer_border[iter->first] = (iter == id_lane_iter1) ? lane_width : lane_width + id_to_outer_border.at(std::prev(iter)->first);
    }

    /* iterate from id #0 towards -inf */
    std::map<int, std::shared_ptr<Lane>>::const_reverse_iterator r_id_lane_iter_1(id_lane_iter0);
    for (auto r_iter = r_id_lane_iter_1; r_iter != lanesection->id_to_lane.rend(); r_iter++)
    {
        const double lane_width = r_iter->second->lane_width.get(s - lanesection->s0);
        id_to_outer_border[r_iter->first] =
            (r_iter == r_id_lane_iter_1) ? -lane_width : -lane_width + id_to_outer_border.at(std::prev(r_iter)->first);
    }

    const double t_offset = this->lane_offset.get(s);
    for (auto& id_border : id_to_outer_border)
        id_border.second += t_offset;

    id_to_outer_border[0] = t_offset;

    return id_to_outer_border;
}

Vec3D Road::get_xyz(double s, double t, double z) const
{
    const Mat3D trans_mat = this->get_transformation_matrix(s);
    const Vec3D xyz = MatVecMultiplication(trans_mat, Vec3D{t, z, 1});

    return xyz;
}

Vec3D Road::get_surface_pt(double s, double t) const
{
    std::shared_ptr<const LaneSection> lanesection = this->get_lanesection(s);
    if (!lanesection || t == 0)
        return this->get_xyz(s, t, 0.0);

    std::map<int, double> lane_id_to_outer_brdr = this->get_lane_borders(s);

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