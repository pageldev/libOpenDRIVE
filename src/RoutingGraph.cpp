#include "RoutingGraph.h"

#include <utility>

namespace odr
{

RoutingGraphEdge::RoutingGraphEdge(LaneKey from, LaneKey to, double length) : from(from), to(to), length(length) {}

void RoutingGraph::add_edge(const RoutingGraphEdge& edge)
{
    this->edges.insert(edge);
    this->successors[edge.from].insert(edge.to);
    this->predecessors[edge.to].insert(edge.from);
}

const std::unordered_set<LaneKey>* RoutingGraph::get_lane_successors(const LaneKey& lane) const
{
    auto successors_iter = this->successors.find(lane);
    if (successors_iter == this->successors.end())
        return nullptr;
    return &(successors_iter->second);
}

std::unordered_set<LaneKey>* RoutingGraph::get_lane_successors(const LaneKey& lane)
{
    return const_cast<std::unordered_set<LaneKey>*>(static_cast<const RoutingGraph&>(*this).get_lane_successors(lane));
}

const std::unordered_set<LaneKey>* RoutingGraph::get_lane_predecessors(const LaneKey& lane) const
{
    auto predecessors_iter = this->predecessors.find(lane);
    if (predecessors_iter == this->predecessors.end())
        return nullptr;
    return &(predecessors_iter->second);
}

std::unordered_set<LaneKey>* RoutingGraph::get_lane_predecessors(const LaneKey& lane)
{
    return const_cast<std::unordered_set<LaneKey>*>(static_cast<const RoutingGraph&>(*this).get_lane_predecessors(lane));
}

} // namespace odr