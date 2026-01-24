#pragma once
#include "LaneValidityRecord.h"
#include "Mesh.h"

#include <optional>
#include <string>
#include <vector>

namespace odr
{

struct RoadSignal
{
    RoadSignal(std::string                id,
               std::string                road_id,
               double                     s0,
               double                     t0,
               double                     zOffset,
               bool                       is_dynamic,
               std::string                type,
               std::string                subtype,
               std::string                orientation,
               std::optional<double>      value,
               std::optional<double>      height,
               std::optional<double>      width,
               std::optional<double>      hOffset,
               std::optional<double>      pitch,
               std::optional<double>      roll,
               std::optional<std::string> name,
               std::optional<std::string> unit,
               std::optional<std::string> text,
               std::optional<std::string> country);

    static constexpr double Thickness = 0.2;
    static constexpr double DefaultWidth = 0.6;
    static constexpr double DefaultHeight = 0.6;

    static Mesh3D get_box(double width, double length, double height);

    std::string id;
    std::string road_id;

    double s0;
    double t0;
    double zOffset;
    bool   is_dynamic;

    std::string type;
    std::string subtype;
    std::string orientation;

    std::optional<double> value;
    std::optional<double> height;
    std::optional<double> width;
    std::optional<double> hOffset;
    std::optional<double> pitch;
    std::optional<double> roll;

    std::optional<std::string> name;
    std::optional<std::string> unit;
    std::optional<std::string> text;
    std::optional<std::string> country;

    std::vector<LaneValidityRecord> lane_validities;
};

} // namespace odr
