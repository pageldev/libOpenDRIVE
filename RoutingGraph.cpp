#include "RoutingGraph.h"

namespace odr
{

void RoutingGraph::add_edge(const RoutingGraphEdge& edge)
{
    this->edges.push_back(edge);
    this->successors[edge.start].insert(edge.end);
    this->predecessors[edge.end].insert(edge.start);
}

} // namespace odr