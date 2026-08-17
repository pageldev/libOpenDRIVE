#pragma once
#include "libodr/Geometries/CubicSpline.h"
#include "libodr/Lane.h"
#include "libodr/LaneSection.h"
#include "libodr/Math.hpp"
#include "libodr/OdrNode.h"
#include "libodr/RefLine.h"
#include "libodr/RoadObject.h"
#include "libodr/RoadSignal.h"

#include <map>
#include <optional>
#include <string>

namespace odr
{

struct Lane;

struct Crossfall
{
    enum class Side
    {
        Both,
        Left,
        Right
    };

    Crossfall() = default;

    struct Record
    {
        CubicPoly poly;
        Side      side = Side::Both;
    };

    double get(double s, bool on_left_side) const;

    std::map<double, Record> records;
};

struct RoadLink : public OdrNode
{
    enum class Type
    {
        Road,
        Junction
    };

    enum class ContactPoint
    {
        Start,
        End
    };

    RoadLink(const std::string& id, const std::string& type_str, std::optional<ContactPoint> contact_point);

    std::string id;
    Type        type;

    std::optional<ContactPoint> contact_point;
};

struct Speed : public OdrNode
{
    Speed(const std::string& max, const std::string& unit);

    std::string max = "";
    std::string unit = "";
};

class Road : public OdrNode
{
public:
    enum class TrafficRule
    {
        LHT,
        RHT
    };

    Road(const std::string&         id,
         double                     length,
         const std::string&         junction,
         std::optional<TrafficRule> traffic_rule = std::nullopt,
         std::optional<std::string> name = std::nullopt);

    const LaneSection* get_lane_section(double s) const;
    LaneSection*       get_lane_section(double s);

    Vec3D
    get_xyz(double s, double t, double h, Vec3D* e_s = nullptr, Vec3D* e_t = nullptr, Vec3D* e_h = nullptr, bool allow_extrapolate = true) const;
    Vec3D get_surface_pt(double s, double t, Vec3D* vn = nullptr, bool allow_extrapolate = true) const;

    std::string id;
    double      length;
    std::string junction;

    std::optional<TrafficRule> traffic_rule;
    std::optional<std::string> name;

    std::optional<RoadLink> predecessor;
    std::optional<RoadLink> successor;

    CubicProfile lane_offset;
    CubicProfile superelevation;
    Crossfall    crossfall;
    RefLine      ref_line;

    std::map<double, LaneSection>     s_to_lane_section;
    std::map<double, std::string>     s_to_type;
    std::map<double, Speed>           s_to_speed;
    std::map<std::string, RoadObject> id_to_object;
    std::map<std::string, RoadSignal> id_to_signal;
};

} // namespace odr
