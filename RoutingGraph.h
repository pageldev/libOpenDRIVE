#pragma once
#include "Lanes.h"

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>
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

struct RoutingGraphEdge
{
    RoutingGraphEdge(RoutingGraphVertex from, RoutingGraphVertex to);
    bool operator==(const RoutingGraphEdge& other) const;

    RoutingGraphVertex from;
    RoutingGraphVertex to;
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

template<>
struct std::hash<odr::RoutingGraphEdge>
{
    std::size_t operator()(const odr::RoutingGraphEdge& e) const
    {
        return ((std::hash<odr::RoutingGraphVertex>()(e.from) ^ (std::hash<odr::RoutingGraphVertex>()(e.to) << 1)) >> 1);
    }
};

namespace odr
{

using RoutingSequentMap = std::unordered_map<RoutingGraphVertex, std::unordered_set<RoutingGraphVertex>>;
using RoutingEdgeSet = std::unordered_set<RoutingGraphEdge>;

class RoutingGraph
{
public:
    RoutingGraph() = default;
    virtual ~RoutingGraph() = default;

    void add_edge(const RoutingGraphEdge& edge);

    const RoutingEdgeSet&    get_edges() const;
    const RoutingSequentMap& get_successors() const;
    const RoutingSequentMap& get_predecessors() const;

private:
    RoutingEdgeSet    edges;
    RoutingSequentMap successors;
    RoutingSequentMap predecessors;
};

} // namespace odr