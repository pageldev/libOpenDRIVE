#include "libodr/Road.h"

#include "libodr/Lane.h"
#include "libodr/RefLine.h"
#include "libodr/Utils.hpp"

#include "magic_enum/magic_enum.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace odr
{

double Crossfall::get(double s, bool on_left_side) const
{
    if (this->records.empty())
        return 0;

    auto target_record_iter = this->records.upper_bound(s);
    if (target_record_iter != this->records.begin())
        target_record_iter--;

    const Record& record = target_record_iter->second;

    if (on_left_side && record.side == Side::Right)
        return 0;
    else if (!on_left_side && record.side == Side::Left)
        return 0;

    return record.poly.evaluate(s);
}

RoadLink::RoadLink(const std::string& id, const std::string& type_str, std::optional<ContactPoint> contact_point) :
    id(id), contact_point(contact_point)
{
    std::optional<Type> type = magic_enum::enum_cast<Type>(type_str, magic_enum::case_insensitive);
    require_or_throw(type.has_value(), "road link type '{}' is invalid", type_str);
    if (*type == Type::Road)
        require_or_throw(contact_point.has_value(), "a road link of type 'road' requires a contact point");
    this->type = *type;
}

Speed::Speed(const std::string& max, const std::string& unit) : max(max), unit(unit) {}

Road::Road(
    const std::string& id, double length, const std::string& junction, std::optional<TrafficRule> traffic_rule, std::optional<std::string> name) :
    id(id), length(length), junction(junction), traffic_rule(traffic_rule), name(name), ref_line(length)
{
    require_or_throw(length > 0, "length must be greater than 0 (got {})", length);
}

const LaneSection* Road::get_lane_section(double s) const
{
    require_or_throw(!(this->s_to_lane_section.empty()), "road has no lane sections");

    auto lane_section_iter = this->s_to_lane_section.upper_bound(s); // first element > s
    if (lane_section_iter != this->s_to_lane_section.begin())
        lane_section_iter--;
    require_or_throw(s >= lane_section_iter->first, "s must not be before lane section start {} (got {})", lane_section_iter->first, s);

    return &lane_section_iter->second;
}

LaneSection* Road::get_lane_section(double s)
{
    return const_cast<LaneSection*>(static_cast<const Road&>(*this).get_lane_section(s));
}

Vec3D Road::get_xyz(double s, double t, double h, Vec3D* _e_s, Vec3D* _e_t, Vec3D* _e_h, bool allow_extrapolate) const
{
    require_or_throw(allow_extrapolate || (s >= 0 && s <= this->length), "s must be in road range [0, {}] (got {})", this->length, s);
    const double s_clamped = std::min(std::max(s, 0.0), this->length);
    const Vec3D  s_vec = this->ref_line.derivative(s_clamped);
    const Vec3D  e_s = normalize(s_vec);

    const Vec3D e_t_base{-e_s[1], e_s[0], 0.0}; // flat in xy-plane and perpendicular to e_s
    const Vec3D e_h_base = crossProduct(e_s, e_t_base);

    // Rodrigues rotation of e_t_base around e_s by theta; simplified since dot(k,v)=0 and cross(k,v)=e_h_base
    const double theta = this->superelevation.evaluate(s_clamped).value_or(0.0);
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    const Vec3D  e_t = normalize(Vec3D{cos_theta * e_t_base[0] + sin_theta * e_h_base[0],
                                      cos_theta * e_t_base[1] + sin_theta * e_h_base[1],
                                      cos_theta * e_t_base[2] + sin_theta * e_h_base[2]});

    const Vec3D e_h = normalize(crossProduct(e_s, e_t));
    Vec3D       p0 = this->ref_line.get_xyz(s_clamped);
    if (s != s_clamped) // out of road bounds, linear extrapolate
        p0 = add(p0, mut(s - s_clamped, s_vec));

    const Vec3D xyz{p0[0] + t * e_t[0] + h * e_h[0], p0[1] + t * e_t[1] + h * e_h[1], p0[2] + t * e_t[2] + h * e_h[2]};

    if (_e_s)
        *_e_s = e_s;
    if (_e_t)
        *_e_t = e_t;
    if (_e_h)
        *_e_h = e_h;

    return xyz;
}

Vec3D Road::get_surface_pt(double s, double t, Vec3D* vn, bool allow_extrapolate) const
{
    const double       s_clamped = std::min(std::max(s, 0.0), this->length);
    const LaneSection* lane_section = this->get_lane_section(s_clamped);
    const int          lane_id = lane_section->get_lane_id(s_clamped, t);
    const Lane&        lane = lane_section->id_to_lane.at(lane_id);
    return lane.get_surface_pt(s, t, vn, allow_extrapolate);
}

} // namespace odr
