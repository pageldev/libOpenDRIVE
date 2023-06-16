#pragma once
#include "LaneValidityRecord.h"
#include "Math.hpp"
#include "Mesh.h"
#include "XmlNode.h"

#include <string>
#include <vector>

namespace odr
{

struct RoadObjectRepeat : public XmlNode
{
    RoadObjectRepeat(double s0,
                     double length,
                     double distance,
                     double t_start,
                     double t_end,
                     double width_start,
                     double width_end,
                     double height_start,
                     double height_end,
                     double z_offset_start,
                     double z_offset_end);

    double s0 = 0;
    double length = 0;
    double distance = 0;
    double t_start = 0;
    double t_end = 0;
    double width_start = 0;
    double width_end = 0;
    double height_start = 0;
    double height_end = 0;
    double z_offset_start = 0;
    double z_offset_end = 0;
};

struct RoadObjectCorner : public XmlNode
{
    enum Type
    {
        Type_Local_RelZ, // z relative to road’s reference line
        Type_Local_AbsZ, // absolute z value
        Type_Road
    };

    RoadObjectCorner(int id, Vec3D pt, double height, Type type);

    int    id = 0;
    Vec3D  pt;
    double height = 0;
    Type   type = Type_Road;
};

struct RoadObjectOutline : public XmlNode
{
    RoadObjectOutline(int id, std::string fill_type, std::string lane_type, bool outer, bool closed);

    int         id = 0;
    std::string fill_type = "";
    std::string lane_type = "";
    bool        outer = true;
    bool        closed = true;

    std::vector<RoadObjectCorner> outline;
};

struct RoadObject : public XmlNode
{
    RoadObject(std::string road_id,
               std::string id,
               double      s0,
               double      t0,
               double      z0,
               double      length,
               double      valid_length,
               double      width,
               double      radius,
               double      height,
               double      hdg,
               double      pitch,
               double      roll,
               std::string type,
               std::string name,
               std::string orientation,
               std::string subtype,
               bool        is_dynamic);

    static Mesh3D get_cylinder(const double eps, const double radius, const double height);
    static Mesh3D get_box(const double width, const double length, const double height);

    std::string road_id = "";

    std::string id = "";
    std::string type = "";
    std::string name = "";
    std::string orientation = "";
    std::string subtype = "";

    double s0 = 0;
    double t0 = 0;
    double z0 = 0;
    double length = 0;
    double valid_length = 0;
    double width = 0;
    double radius = 0;
    double height = 0;
    double hdg = 0;
    double pitch = 0;
    double roll = 0;
    bool   is_dynamic = false;

    std::vector<RoadObjectRepeat>   repeats;
    std::vector<RoadObjectOutline>  outlines;
    std::vector<LaneValidityRecord> lane_validities;
};

} // namespace odr