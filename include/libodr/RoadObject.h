#pragma once
#include "libodr/LaneValidity.h"
#include "libodr/Math.hpp"
#include "libodr/Mesh.h"

#include <optional>
#include <string>
#include <vector>

namespace odr
{

struct RoadObjectRepeat
{
    RoadObjectRepeat(double                s,
                     double                length,
                     double                distance,
                     std::optional<double> t_start = std::nullopt,
                     std::optional<double> t_end = std::nullopt,
                     std::optional<double> height_start = std::nullopt,
                     std::optional<double> height_end = std::nullopt,
                     std::optional<double> z_offset_start = std::nullopt,
                     std::optional<double> z_offset_end = std::nullopt,
                     std::optional<double> width_start = std::nullopt,
                     std::optional<double> width_end = std::nullopt);

    double s;
    double length; // length of repeat area, not object
    double distance;

    // required but often treated as optional
    std::optional<double> t_start;
    std::optional<double> t_end;
    std::optional<double> height_start;
    std::optional<double> height_end;
    std::optional<double> z_offset_start;
    std::optional<double> z_offset_end;

    std::optional<double> width_start;
    std::optional<double> width_end;
};

struct RoadObjectCorner
{
    enum class Type
    {
        Local_RelZ, // z relative to road’s reference line
        Local_AbsZ, // absolute z value
        Road
    };

    RoadObjectCorner(Vec3D pt, double height, Type type, std::optional<int> id = std::nullopt);

    Vec3D  pt;
    double height;
    Type   type;

    std::optional<int> id;
};

struct RoadObjectOutline
{
    RoadObjectOutline(std::optional<int>         id = std::nullopt,
                      std::optional<std::string> fill_type = std::nullopt,
                      std::optional<std::string> lane_type = std::nullopt,
                      std::optional<bool>        outer = std::nullopt,
                      std::optional<bool>        closed = std::nullopt);

    std::optional<int>         id;
    std::optional<std::string> fill_type;
    std::optional<std::string> lane_type;
    std::optional<bool>        outer;
    std::optional<bool>        closed;

    std::vector<RoadObjectCorner> outline;
};

struct RoadObject
{
    enum class Orientation
    {
        None,
        Positive,
        Negative
    };

    RoadObject(const std::string&         id,
               std::optional<double>      s = std::nullopt,
               std::optional<double>      t = std::nullopt,
               std::optional<double>      z_offset = std::nullopt,
               std::optional<double>      length = std::nullopt,
               std::optional<double>      valid_length = std::nullopt,
               std::optional<double>      width = std::nullopt,
               std::optional<double>      radius = std::nullopt,
               std::optional<double>      height = std::nullopt,
               std::optional<double>      hdg = std::nullopt,
               std::optional<double>      pitch = std::nullopt,
               std::optional<double>      roll = std::nullopt,
               std::optional<std::string> type = std::nullopt,
               std::optional<std::string> name = std::nullopt,
               std::optional<std::string> subtype = std::nullopt,
               std::optional<Orientation> orientation = std::nullopt,
               std::optional<bool>        is_dynamic = std::nullopt);

    static Mesh3D get_cylinder(double eps, double radius, double height);
    static Mesh3D get_cube(double width, double length, double height);

    std::string id;

    // can be superseded by Object Repeat Record (Rev. 1.4, 5.3.8.1.1)
    std::optional<double> s;
    std::optional<double> t;
    std::optional<double> z_offset;

    std::optional<double> length;       // physical length vs.
    std::optional<double> valid_length; // validity range, has no influence on shape
    std::optional<double> width;
    std::optional<double> radius;
    std::optional<double> height;
    std::optional<double> hdg;
    std::optional<double> pitch;
    std::optional<double> roll;

    std::optional<std::string> type;
    std::optional<std::string> name;
    std::optional<std::string> subtype;

    std::optional<Orientation> orientation;

    std::optional<bool> is_dynamic;

    std::vector<RoadObjectRepeat>  repeats;
    std::vector<RoadObjectOutline> outlines;
    std::vector<LaneValidity>      lane_validities;
};

} // namespace odr
