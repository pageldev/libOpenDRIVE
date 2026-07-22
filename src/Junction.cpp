#include "libodr/Junction.h"
#include "libodr/Utils.hpp"
#include <string>

namespace odr
{

JunctionLaneLink::JunctionLaneLink(int from, int to) : from(from), to(to) {}

JunctionConnection::JunctionConnection(const std::string& id,
                                       const std::string& incoming_road,
                                       const std::string& connecting_road,
                                       const std::string& contact_point_str) :
    id(id), incoming_road(incoming_road), connecting_road(connecting_road)
{
    std::optional<ContactPoint> contact_point = magic_enum::enum_cast<ContactPoint>(contact_point_str, magic_enum::case_insensitive);
    require_or_throw(contact_point.has_value(), "invalid junction connection contact type '{}'", contact_point_str);
    this->contact_point = *contact_point;
}

JunctionPriority::JunctionPriority(const std::string& high, const std::string& low) : high(high), low(low) {}

JunctionController::JunctionController(const std::string& id, std::optional<std::string> type, std::optional<int64_t> sequence) :
    id(id), type(type), sequence(sequence)
{
    require_or_throw(!sequence || sequence >= 0, "sequence < 0");
}

Junction::Junction(const std::string& id, std::optional<std::string> name) : id(id), name(name) {}

} // namespace odr