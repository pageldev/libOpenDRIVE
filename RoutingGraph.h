#pragma once
#include "Lanes.h"

#include <cstdint>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

namespace odr
{

struct RoutingGraphVertex
{
    RoutingGraphVertex(std::string road_id, double lane_section_s0, int lane_id);
    bool operator<(const RoutingGraphVertex& other) const;
    bool operator==(const RoutingGraphVertex& other) const;

    std::string road_id = "";
    double      lane_section_s0 = 0.0;
    int         lane_id = 0;
};

} // namespace odr

template<>
struct std::hash<odr::RoutingGraphVertex>
{
    std::size_t operator()(const odr::RoutingGraphVertex& v) const
    {
        return ((std::hash<string>()(v.road_id) ^ (std::hash<double>()(v.lane_section_s0) << 1)) >> 1) ^ (std::hash<int>()(v.lane_id) << 1);
    }
};

namespace odr
{

struct RoutingGraphEdge
{
    RoutingGraphEdge(RoutingGraphVertex from, RoutingGraphVertex to);

    RoutingGraphVertex from;
    RoutingGraphVertex to;
};

class RoutingGraph
{
public:
    RoutingGraph() = default;
    virtual ~RoutingGraph() = default;

    void add_edge(const RoutingGraphEdge& edge);

    std::vector<RoutingGraphEdge> edges;

    std::unordered_map<RoutingGraphVertex, std::set<RoutingGraphVertex>> successors;
    std::unordered_map<RoutingGraphVertex, std::set<RoutingGraphVertex>> predecessors;
};

} // namespace odr