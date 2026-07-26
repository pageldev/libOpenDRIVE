#include "libodr/RoadMark.h"
#include "libodr/Utils.hpp"
#include <cmath>

namespace odr
{

RoadMarkLine::RoadMarkLine(double                     sOffset,
                           double                     tOffset,
                           double                     length,
                           std::optional<double>      width,
                           std::optional<double>      space,
                           std::optional<std::string> color,
                           std::optional<std::string> rule) :
    sOffset(sOffset), tOffset(tOffset), length(length), width(width), space(space), color(color), rule(rule)
{
    require_or_throw(sOffset >= 0, "sOffset must be greater than or equal to 0 (got {})", sOffset);
    require_or_throw(!std::isnan(tOffset), "tOffset must not be NaN");
    require_or_throw(length >= 0, "length must be greater than or equal to 0 (got {})", length);
    require_or_throw(!width || width > 0, "width must be greater than 0");
    require_or_throw(!space || space >= 0, "space must be greater than or equal to 0");
}

RoadMarkType::RoadMarkType(const std::string& name, std::optional<double> width) : name(name), width(width)
{
    require_or_throw(!width || width > 0, "width must be greater than 0");
}

RoadMark::RoadMark(double                     s_offset,
                   const std::string&         type,
                   const std::string&         color,
                   std::optional<double>      width,
                   std::optional<double>      height,
                   std::optional<std::string> weight,
                   std::optional<std::string> material,
                   std::optional<std::string> lane_change) :
    s_offset(s_offset), type(type), color(color), width(width), height(height), weight(weight), material(material), lane_change(lane_change)
{
    require_or_throw(s_offset >= 0, "sOffset must be greater than or equal to 0 (got {})", s_offset);
    require_or_throw(!width || width >= 0, "width must be greater than or equal to 0");
    require_or_throw(!height || height > 0, "height must be greater than 0");
}

SingleRoadMark::SingleRoadMark(double s0, double s1, double t, double width, const std::string& type) noexcept :
    s0(s0), s1(s1), t(t), width(width), type(type)
{
}

} // namespace odr