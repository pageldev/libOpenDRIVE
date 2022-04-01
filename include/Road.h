#pragma once

#include "Geometries/CubicSpline.h"
#include "LaneSection.h"
#include "Math.hpp"
#include "RoadObject.h"
#include "Utils.hpp"
#include "XmlNode.h"

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace odr
{
struct RefLine;

struct Crossfall : public CubicSpline
{
    enum Side
    {
        Both,
        Left,
        Right
    };

    Crossfall() = default;
    double get_crossfall(double s, bool on_left_side) const;

    std::map<double, Side> sides;
};

struct RoadLink : public XmlNode
{
    enum class ContactPoint
    {
        None,
        Start,
        End
    };

    enum class Type
    {
        None,
        Road,
        Junction
    };

    std::string  id = "";
    Type         type = Type::None;
    ContactPoint contact_point = ContactPoint::None;
};

struct RoadNeighbor : public XmlNode
{
    std::string id = "";
    std::string side = "";
    std::string direction = "";
};

struct SpeedRecord : public XmlNode
{
    std::string max = "";
    std::string unit = "";
};

class Road : public XmlNode, public std::enable_shared_from_this<Road>
{
public:
    Road() = default;
    virtual ~Road() = default;

    ConstLaneSectionSet get_lanesections() const;
    LaneSectionSet      get_lanesections();

    ConstRoadObjectSet get_road_objects() const;
    RoadObjectSet      get_road_objects();

    std::shared_ptr<const LaneSection> get_lanesection(double s) const;
    std::shared_ptr<LaneSection>       get_lanesection(double s);

    Vec3D get_xyz(double s, double t, double h, Vec3D* e_s = nullptr, Vec3D* e_t = nullptr, Vec3D* e_h = nullptr) const;

    double      length = 0;
    std::string id;
    std::string junction;
    std::string name;

    RoadLink                  predecessor;
    RoadLink                  successor;
    std::vector<RoadNeighbor> neighbors;

    CubicSpline              lane_offset;
    CubicSpline              superelevation;
    Crossfall                crossfall;
    std::shared_ptr<RefLine> ref_line;

    std::map<double, std::shared_ptr<LaneSection>>     s_to_lanesection;
    std::map<double, std::string>                      s_to_type;
    std::map<double, SpeedRecord>                      s_to_speed;
    std::map<std::string, std::shared_ptr<RoadObject>> id_to_object;
};

using ConstRoadSet = std::set<std::shared_ptr<const Road>, SharedPtrCmp<const Road, std::string, &Road::id>>;
using RoadSet = std::set<std::shared_ptr<Road>, SharedPtrCmp<Road, std::string, &Road::id>>;

} // namespace odr