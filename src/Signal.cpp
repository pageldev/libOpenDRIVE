#include "Signal.h"

namespace odr
{
Signal::Signal(std::string road_id,
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
               std::string text) :
    road_id(road_id),
    id(id), name(name), s0(s0), t0(t0), is_dynamic(is_dynamic), zOffset(zOffset), value(value), height(height), width(width), hOffset(hOffset),
    pitch(pitch), roll(roll), orientation(orientation), country(country), type(type), subtype(subtype), unit(unit), text(text)
{
}
} // namespace odr
