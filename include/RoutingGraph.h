#pragma once
#include "Lane.h"

#include <cstddef>
#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>

namespace odr
{

struct RoutingGraphEdge
{
    RoutingGraphEdge(LaneKey from, LaneKey to, double length);

    LaneKey from;
    LaneKey to;
    double  length;
};

} // namespace odr

template<>
struct std::hash<odr::RoutingGraphEdge>
{
    std::size_t operator()(const odr::RoutingGraphEdge& e) const
    {
        return ((std::hash<odr::LaneKey>()(e.from) ^ (std::hash<odr::LaneKey>()(e.to) << 1)) >> 1);
    }
};

template<>
struct std::equal_to<odr::RoutingGraphEdge>
{
    std::size_t operator()(const odr::RoutingGraphEdge& lhs, const odr::RoutingGraphEdge& rhs) const
    {
        return std::equal_to<odr::LaneKey>{}(lhs.from, rhs.from) && std::equal_to<odr::LaneKey>{}(lhs.to, rhs.to);
    }
};

namespace odr
{

class RoutingGraph
{
public:
    RoutingGraph() = default;

    void add_edge(const RoutingGraphEdge& edge);

    const std::unordered_set<LaneKey>* get_lane_successors(const LaneKey& lane) const;
    std::unordered_set<LaneKey>*       get_lane_successors(const LaneKey& lane);
    const std::unordered_set<LaneKey>* get_lane_predecessors(const LaneKey& lane) const;
    std::unordered_set<LaneKey>*       get_lane_predecessors(const LaneKey& lane);

    std::unordered_set<RoutingGraphEdge> edges;

    using SequentMap = std::unordered_map<LaneKey, std::unordered_set<LaneKey>>;
    SequentMap successors;
    SequentMap predecessors;
};

} // namespace odr
