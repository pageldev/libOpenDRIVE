#include "Road.h"
#include "RefLine.h"

#include <cmath>
#include <utility>

namespace odr
{
double Crossfall::get_crossfall(double s, bool on_left_side) const
{
    const Poly3 poly = this->get_poly(s);

    if (this->s0_to_poly.size() > 0)
    {
        auto target_poly_iter = this->s0_to_poly.upper_bound(s);
        if (target_poly_iter != this->s0_to_poly.begin())
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

ConstRoadObjectSet Road::get_road_objects() const
{
    ConstRoadObjectSet road_objects;
    for (const auto& id_obj : this->id_to_object)
        road_objects.insert(id_obj.second);

    return road_objects;
}

RoadObjectSet Road::get_road_objects()
{
    RoadObjectSet road_objects;
    for (const auto& id_obj : this->id_to_object)
        road_objects.insert(id_obj.second);

    return road_objects;
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

Vec3D Road::get_xyz(double s, double t, double h, Vec3D* _e_s, Vec3D* _e_t, Vec3D* _e_h) const
{
    const Vec3D  s_vec = this->ref_line->get_grad(s);
    const double theta = this->superelevation.get(s);

    const Vec3D e_s = normalize(s_vec);
    const Vec3D e_t = normalize(Vec3D{std::cos(theta) * -e_s[1] + std::sin(theta) * -e_s[2] * e_s[0],
                                      std::cos(theta) * e_s[0] + std::sin(theta) * -e_s[2] * e_s[1],
                                      std::sin(theta) * (e_s[0] * e_s[0] + e_s[1] * e_s[1])});
    const Vec3D e_h = normalize(crossProduct(s_vec, e_t));
    const Vec3D p0 = this->ref_line->get_xyz(s);
    const Mat3D trans_mat{{{e_t[0], e_h[0], p0[0]}, {e_t[1], e_h[1], p0[1]}, {e_t[2], e_h[2], p0[2]}}};

    const Vec3D xyz = MatVecMultiplication(trans_mat, Vec3D{t, h, 1});

    if (_e_s)
        *_e_s = e_s;
    if (_e_t)
        *_e_t = e_t;
    if (_e_h)
        *_e_h = e_h;

    return xyz;
}

} // namespace odr