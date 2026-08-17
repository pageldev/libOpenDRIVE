#pragma once
#include "libodr/OdrNode.h"

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace odr
{

struct JunctionLaneLink : public OdrNode
{
    JunctionLaneLink(int from, int to);

    int from;
    int to;
};

struct JunctionConnection : public OdrNode
{
    enum class ContactPoint
    {
        None, // for init
        Start,
        End
    };

    JunctionConnection(const std::string& id,
                       const std::string& incoming_road,
                       const std::string& connecting_road,
                       const std::string& contact_point_str);

    std::string  id;
    std::string  incoming_road;
    std::string  connecting_road;
    ContactPoint contact_point; // contact point on the connectingRoad

    std::vector<JunctionLaneLink> lane_links;
};

struct JunctionPriority : public OdrNode
{
    JunctionPriority(const std::string& high, const std::string& low);

    std::string high;
    std::string low;
};

struct JunctionController : public OdrNode
{
    JunctionController(const std::string& id, std::optional<std::string> type = std::nullopt, std::optional<int64_t> sequence = std::nullopt);

    std::string id;

    std::optional<std::string> type;
    std::optional<uint32_t>    sequence;
};

class Junction : public OdrNode
{
public:
    Junction(const std::string& id, std::optional<std::string> name = std::nullopt);

    std::string id;

    std::optional<std::string> name;

    std::map<std::string, JunctionConnection> id_to_connection;
    std::map<std::string, JunctionController> id_to_controller;
    std::vector<JunctionPriority>             priorities;
};

} // namespace odr
