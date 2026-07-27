#pragma once
#include "libodr/LaneValidityRecord.h"
#include "libodr/Mesh.h"
#include "libodr/RoadObject.h"

#include <optional>
#include <string>
#include <vector>

namespace odr
{

struct RoadSignal
{
    RoadSignal(const std::string&         id,
               const std::string&         road_id,
               double                     s,
               double                     t,
               double                     z_offset,
               bool                       is_dynamic,
               const std::string&         type,
               const std::string&         subtype,
               RoadObject::Orientation    orientation,
               std::optional<double>      value = std::nullopt,
               std::optional<double>      height = std::nullopt,
               std::optional<double>      width = std::nullopt,
               std::optional<double>      hOffset = std::nullopt,
               std::optional<double>      pitch = std::nullopt,
               std::optional<double>      roll = std::nullopt,
               std::optional<std::string> name = std::nullopt,
               std::optional<std::string> unit = std::nullopt,
               std::optional<std::string> text = std::nullopt,
               std::optional<std::string> country = std::nullopt);

    static constexpr double Thickness = 0.2;
    static constexpr double DefaultWidth = 0.6;
    static constexpr double DefaultHeight = 0.6;

    static Mesh3D get_box(double width, double length, double height);

    std::string id;
    std::string road_id;

    double s;
    double t;
    double z_offset;
    bool   is_dynamic;

    std::string type;
    std::string subtype;

    RoadObject::Orientation orientation;

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
