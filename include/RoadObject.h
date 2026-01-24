#pragma once
#include "LaneValidityRecord.h"
#include "Math.hpp"
#include "Mesh.h"

#include <optional>
#include <string>
#include <vector>

namespace odr
{

struct RoadObjectRepeat
{
    RoadObjectRepeat(double                s0,
                     double                length,
                     double                distance,
                     double                t_start,
                     double                t_end,
                     double                height_start,
                     double                height_end,
                     double                z_offset_start,
                     double                z_offset_end,
                     std::optional<double> width_start = std::nullopt,
                     std::optional<double> width_end = std::nullopt);

    double s0;
    double length;
    double distance;
    double t_start;
    double t_end;
    double height_start;
    double height_end;
    double z_offset_start;
    double z_offset_end;

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
    RoadObject(std::string                road_id,
               std::string                id,
               double                     s0,
               double                     t0,
               double                     z0,
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
               std::optional<std::string> orientation = std::nullopt,
               std::optional<std::string> subtype = std::nullopt,
               std::optional<bool>        is_dynamic = std::nullopt);

    static Mesh3D get_cylinder(double eps, double radius, double height);
    static Mesh3D get_box(double width, double length, double height);

    std::string road_id;
    std::string id;

    double s0;
    double t0;
    double z0;

    std::optional<double> length;
    std::optional<double> valid_length;
    std::optional<double> width;
    std::optional<double> radius;
    std::optional<double> height;
    std::optional<double> hdg;
    std::optional<double> pitch;
    std::optional<double> roll;

    std::optional<std::string> type;
    std::optional<std::string> name;
    std::optional<std::string> orientation;
    std::optional<std::string> subtype;

    std::optional<bool> is_dynamic;

    std::vector<RoadObjectRepeat>   repeats;
    std::vector<RoadObjectOutline>  outlines;
    std::vector<LaneValidityRecord> lane_validities;
};

} // namespace odr