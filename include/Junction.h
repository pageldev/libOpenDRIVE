#pragma once
#include "Utils.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>

namespace odr
{

struct JunctionLaneLink
{
    JunctionLaneLink(int from, int to);

    int from;
    int to;
};

} // namespace odr

namespace std
{
template<>
struct less<odr::JunctionLaneLink>
{
    bool operator()(const odr::JunctionLaneLink& lhs, const odr::JunctionLaneLink& rhs) const
    {
        return odr::compare_class_members(lhs, rhs, less<void>(), &odr::JunctionLaneLink::from, &odr::JunctionLaneLink::to);
    }
};
} // namespace std

namespace odr
{

struct JunctionConnection
{
    enum class ContactPoint
    {
        None, // needed for init
        Start,
        End
    };

    JunctionConnection(std::string id, std::string incoming_road, std::string connecting_road, ContactPoint contact_point);

    std::string  id = "";
    std::string  incoming_road = "";
    std::string  connecting_road = "";
    ContactPoint contact_point = ContactPoint::None;

    std::set<JunctionLaneLink> lane_links;
};

struct JunctionPriority
{
    JunctionPriority(std::string high, std::string low);

    std::string high = "";
    std::string low = "";
};

} // namespace odr

namespace std
{
template<>
struct less<odr::JunctionPriority>
{
    bool operator()(const odr::JunctionPriority& lhs, const odr::JunctionPriority& rhs) const
    {
        return odr::compare_class_members(lhs, rhs, less<void>(), &odr::JunctionPriority::high, &odr::JunctionPriority::low);
    }
};
} // namespace std

namespace odr
{

struct JunctionController
{
    JunctionController(std::string id, std::optional<std::string> type = std::nullopt, std::optional<int64_t> sequence = std::nullopt);

    std::string id = "";

    std::optional<std::string> type;
    std::optional<uint32_t>    sequence;
};

class Junction
{
public:
    Junction(std::string id, std::optional<std::string> name = std::nullopt);

    std::string id = "";

    std::optional<std::string> name;

    std::map<std::string, JunctionConnection> id_to_connection;
    std::map<std::string, JunctionController> id_to_controller;
    std::set<JunctionPriority>                priorities;
};

} // namespace odr