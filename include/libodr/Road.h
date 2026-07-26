#pragma once
#include "libodr/Geometries/CubicSpline.h"
#include "libodr/Lane.h"
#include "libodr/LaneSection.h"
#include "libodr/Math.hpp"
#include "libodr/Mesh.h"
#include "libodr/RefLine.h"
#include "libodr/RoadMark.h"
#include "libodr/RoadObject.h"
#include "libodr/RoadSignal.h"

#include <map>
#include <optional>
#include <set>
#include <stdint.h>
#include <string>
#include <vector>

namespace odr
{

struct Lane;
struct RoadMark;

struct Crossfall : public CubicProfile // extends cubic with 'side' attribute
{
    enum class Side
    {
        Both,
        Left,
        Right
    };

    Crossfall() = default;

    double get(const double s, const bool on_left_side) const;

    std::map<double, Side> s_to_side;
};

struct RoadLink
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

struct SpeedRecord
{
    SpeedRecord(const std::string& max, const std::string& unit);

    std::string max = "";
    std::string unit = "";
};

class Road
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

    std::vector<LaneSection> get_lanesections() const;
    std::vector<RoadObject>  get_road_objects() const;
    std::vector<RoadSignal>  get_road_signals() const;

    double      get_lanesection_s0(const double s) const;
    LaneSection get_lanesection(const double s) const;

    double get_lanesection_end(const LaneSection& lanesection) const;
    double get_lanesection_end(const double lanesection_s0) const;
    double get_lanesection_length(const LaneSection& lanesection) const;
    double get_lanesection_length(const double lanesection_s0) const;

    Vec3D get_xyz(const double s,
                  const double t,
                  const double h,
                  Vec3D*       e_s = nullptr,
                  Vec3D*       e_t = nullptr,
                  Vec3D*       e_h = nullptr,
                  const bool   allow_extrapolate = true) const;
    Vec3D get_surface_pt(double s, const double t, Vec3D* vn = nullptr, bool allow_extrapolate = true) const;

    Line3D get_lane_border_line(const LaneKey& lane_key, double s_start, double s_end, double eps, bool outer = true) const;
    Line3D get_lane_border_line(const LaneKey& lane_key, double eps, bool outer = true) const;

    Mesh3D get_lane_mesh(const LaneKey& lane_key, double s_start, double s_end, double eps, std::vector<uint32_t>* outline_indices = nullptr) const;
    Mesh3D get_lane_mesh(const LaneKey& lane_key, double eps, std::vector<uint32_t>* outline_indices = nullptr) const;

    Mesh3D get_roadmark_mesh(const LaneKey& lane_key, const SingleRoadMark& roadmark, double eps, bool enforce_road_bounds = false) const;
    Mesh3D get_road_signal_mesh(const RoadSignal& road_signal, bool enforce_road_bounds = false) const;
    Mesh3D get_road_object_mesh(const RoadObject&         road_object,
                                double                    eps,
                                double                    default_h = 0,
                                double                    default_z = 0,
                                bool                      enforce_road_bounds = false,
                                std::vector<std::string>* errors = nullptr) const;

    std::set<double> approximate_lane_border_linear(const LaneKey& lane_key, double s_start, double s_end, double eps, bool outer = true) const;
    std::set<double> approximate_lane_border_linear(const LaneKey& lane_key, double eps, bool outer = true) const;

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

    std::map<double, LaneSection>     s_to_lanesection;
    std::map<double, std::string>     s_to_type;
    std::map<double, SpeedRecord>     s_to_speed;
    std::map<std::string, RoadObject> id_to_object;
    std::map<std::string, RoadSignal> id_to_signal;
};

} // namespace odr
