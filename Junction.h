#pragma once
#include "XmlNode.h"

#include <map>
#include <string>
#include <vector>

namespace odr
{

struct JunctionLaneLink
{
    int from = 0;
    int to = 0;
};

struct JunctionConnection
{
    enum class ContactPoint
    {
        None,
        Start,
        End
    };

    std::string  id = "";
    std::string  incoming_road = "";
    std::string  connecting_road = "";
    ContactPoint contact_point = ContactPoint::None;

    std::vector<JunctionLaneLink> lane_links;
};

struct JunctionPriority
{
    std::string high = "";
    std::string low = "";
};

struct JunctionController
{
    std::string id = "";
    std::string type = "";
    uint32_t    sequence = 0;
};

class Junction : public XmlNode, public std::enable_shared_from_this<Junction>
{
public:
    Junction() = default;
    virtual ~Junction() = default;

    std::string name = "";
    std::string id = "";

    std::map<std::string, JunctionConnection> connections;
    std::map<std::string, JunctionController> controllers;
    std::vector<JunctionPriority>             priorities;
};

} // namespace odr
