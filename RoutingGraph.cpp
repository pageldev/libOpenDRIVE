#include "RoutingGraph.h"

namespace odr
{

RoutingGraphVertex::RoutingGraphVertex(std::string road_id, double lane_section_s0, int lane_id) :
    road_id(road_id), lane_section_s0(lane_section_s0), lane_id(lane_id)
{
}

bool RoutingGraphVertex::operator<(const RoutingGraphVertex& other) const
{
    if (this->road_id != other.road_id)
        return this->road_id < other.road_id;
    if (this->lane_section_s0 != other.lane_section_s0)
        return this->lane_section_s0 < other.lane_section_s0;
    return this->lane_id < other.lane_id;
}

bool RoutingGraphVertex::operator==(const RoutingGraphVertex& other) const
{
    if (this->road_id == other.road_id && this->lane_section_s0 == other.lane_section_s0 && this->lane_id == other.lane_id)
        return true;
    return false;
}

RoutingGraphEdge::RoutingGraphEdge(RoutingGraphVertex from, RoutingGraphVertex to) : from(from), to(to) {}

bool RoutingGraphEdge::operator==(const RoutingGraphEdge& other) const
{
    if (this->from == other.from && this->to == other.to)
        return true;
    return false;
}

void RoutingGraph::add_edge(const RoutingGraphEdge& edge)
{
    this->edges.insert(edge);
    this->successors[edge.from].insert(edge.to);
    this->predecessors[edge.to].insert(edge.from);
}

const RoutingEdgeSet& RoutingGraph::get_edges() const { return this->edges; }

const RoutingSequentMap& RoutingGraph::get_successors() const { return this->successors; }

const RoutingSequentMap& RoutingGraph::get_predecessors() const { return this->predecessors; }

} // namespace odr