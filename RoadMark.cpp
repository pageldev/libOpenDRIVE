#include "RoadMark.h"

namespace odr
{
RoadMarksLine::RoadMarksLine(double width, double length, double space, double tOffset, double sOffset, std::string name, std::string rule) :
    width(width), length(length), space(space), tOffset(tOffset), sOffset(sOffset), name(name), rule(rule)
{
}

RoadMark::RoadMark(double      width,
                   double      height,
                   double      sOffset,
                   std::string type,
                   std::string weight,
                   std::string color,
                   std::string material,
                   std::string laneChange) :
    width(width),
    height(height), sOffset(sOffset), type(type), weight(weight), color(color), material(material), laneChange(laneChange)
{
}

} // namespace odr