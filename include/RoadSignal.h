#pragma once
#include "LaneValidityRecord.h"
#include "Mesh.h"
#include "XmlNode.h"

#include <string>
#include <vector>

namespace odr
{

struct RoadSignal : public XmlNode
{
    RoadSignal(std::string road_id,
               std::string id,
               std::string name,
               double      s0,
               double      t0,
               bool        is_dynamic,
               double      zOffset,
               double      value,
               double      height,
               double      width,
               double      hOffset,
               double      pitch,
               double      roll,
               std::string orientation,
               std::string country,
               std::string type,
               std::string subtype,
               std::string unit,
               std::string text);

    static Mesh3D get_box(const double width, const double length, const double height);

    std::string road_id = "";
    std::string id = "";
    std::string name = "";
    double      s0 = 0;
    double      t0 = 0;
    bool        is_dynamic = false;
    double      zOffset = 0;
    double      value = 0;
    double      height = 0;
    double      width = 0;
    double      hOffset = 0;
    double      pitch = 0;
    double      roll = 0;
    std::string orientation = "";
    std::string country = "";
    std::string type = "";
    std::string subtype = "";
    std::string unit = "";
    std::string text = "";

    std::vector<LaneValidityRecord> lane_validities;
};

} // namespace odr
