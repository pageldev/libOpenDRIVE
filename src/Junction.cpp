#include "Junction.h"
#include "Utils.hpp"
#include <string>

namespace odr
{

JunctionLaneLink::JunctionLaneLink(int from, int to) : from(from), to(to) {}

JunctionConnection::JunctionConnection(std::string id, std::string incoming_road, std::string connecting_road, ContactPoint contact_point) :
    id(id), incoming_road(incoming_road), connecting_road(connecting_road), contact_point(contact_point)
{
}

JunctionPriority::JunctionPriority(std::string high, std::string low) : high(high), low(low) {}

JunctionController::JunctionController(std::string id, std::optional<std::string> type, std::optional<int64_t> sequence) :
    id(id), type(type), sequence(sequence)
{
    require_or_throw(!sequence || sequence >= 0, "sequence < 0");
}

Junction::Junction(std::string id, std::optional<std::string> name) : id(id), name(name) {}

} // namespace odr