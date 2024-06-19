#pragma once
#include "Geometries/CubicSpline.h"
#include "LaneSection.h"
#include "Math.hpp"
#include "Mesh.h"
#include "RefLine.h"
#include "RoadObject.h"
#include "RoadSignal.h"
#include "XmlNode.h"

#include <cstddef>
#include <map>
#include <set>
#include <vector>
#include <optional>

namespace odr
{

struct Lane;
struct RoadMark;

struct Crossfall : public CubicSpline
{
    enum Side
    {
        Side_Both,
        Side_Left,
        Side_Right
    };

    Crossfall() = default;

    double get_crossfall(const double s, const bool on_left_side) const;

    std::map<double, Side> sides;
};

struct RoadLink : public XmlNode
{
    enum ContactPoint
    {
        ContactPoint_None,
        ContactPoint_Start,
        ContactPoint_End
    };

    enum Type
    {
        Type_None,
        Type_Road,
        Type_Junction
    };

    RoadLink() = default;
    RoadLink(std::string id, Type type, ContactPoint contact_point);

    std::string  id = "";
    Type         type = Type_None;
    ContactPoint contact_point = ContactPoint_None;
};

struct RoadNeighbor : public XmlNode
{
    RoadNeighbor(std::string id, std::string side, std::string direction);

    std::string id = "";
    std::string side = "";
    std::string direction = "";
};

struct SpeedRecord : public XmlNode
{
    SpeedRecord(std::string max, std::string unit);

    std::string max = "";
    std::string unit = "";
};

class Road : public XmlNode
{
public:
    Road(std::string id, double length, std::string junction, std::string name, bool left_hand_traffic = false);

    std::vector<LaneSection> get_lanesections() const;
    std::vector<RoadObject>  get_road_objects() const;
    std::vector<RoadSignal>  get_road_signals() const;

    double      get_lanesection_s0(const double s) const;
    LaneSection get_lanesection(const double s) const;

    double get_lanesection_end(const LaneSection& lanesection) const;
    double get_lanesection_end(const double lanesection_s0) const;
    double get_lanesection_length(const LaneSection& lanesection) const;
    double get_lanesection_length(const double lanesection_s0) const;

    std::optional<LaneSection> get_next_lanesection(const LaneSection& lanesection) const;
    std::optional<LaneSection> get_previous_lanesection(const LaneSection& lanesection) const;

    Vec3D get_xyz(const double s, const double t, const double h, Vec3D* e_s = nullptr, Vec3D* e_t = nullptr, Vec3D* e_h = nullptr) const;
    Vec3D get_surface_pt(double s, const double t, Vec3D* vn = nullptr) const;

    Line3D get_lane_border_line(const Lane& lane, const double s_start, const double s_end, const double eps, const bool outer = true) const;
    Line3D get_lane_border_line(const Lane& lane, const double eps, const bool outer = true) const;

    Mesh3D get_lane_mesh(
        const Lane& lane, const double s_start, const double s_end, const double eps, std::vector<uint32_t>* outline_indices = nullptr) const;
    Mesh3D get_lane_mesh(const Lane& lane, const double eps, std::vector<uint32_t>* outline_indices = nullptr) const;

    Mesh3D get_roadmark_mesh(const Lane& lane, const RoadMark& roadmark, const double eps) const;
    Mesh3D get_road_signal_mesh(const RoadSignal& road_signal) const;
    Mesh3D get_road_object_mesh(const RoadObject& road_object, const double eps) const;

    std::set<double>
    approximate_lane_border_linear(const Lane& lane, const double s_start, const double s_end, const double eps, const bool outer = true) const;
    std::set<double> approximate_lane_border_linear(const Lane& lane, const double eps, const bool outer = true) const;

    template<typename Point>
    double calculate_curvature(const Point& p1, const Point& p2, const Point& p3) const
    {
        double x1 = p1[0], y1 = p1[1];
        double x2 = p2[0], y2 = p2[1];
        double x3 = p3[0], y3 = p3[1];

        double a = std::sqrt((x2 - x3) * (x2 - x3) + (y2 - y3) * (y2 - y3));
        double b = std::sqrt((x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3));
        double c = std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

        double s = (a + b + c) / 2.0;
        double area = std::fabs(0.5 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)));

        if (area == 0.0)
        {
            return 0.0;
        }

        double curvature = (4.0 * area) / (a * b * c);

        return curvature;
    }

    double      length = 0;
    std::string id = "";
    std::string junction = "";
    std::string name = "";
    bool        left_hand_traffic = false;

    RoadLink                  predecessor;
    RoadLink                  successor;
    std::vector<RoadNeighbor> neighbors;

    CubicSpline lane_offset;
    CubicSpline superelevation;
    Crossfall   crossfall;
    RefLine     ref_line;

    std::map<double, LaneSection>     s_to_lanesection;
    std::map<double, std::string>     s_to_type;
    std::map<double, SpeedRecord>     s_to_speed;
    std::map<std::string, RoadObject> id_to_object;
    std::map<std::string, RoadSignal> id_to_signal;
};

} // namespace odr
