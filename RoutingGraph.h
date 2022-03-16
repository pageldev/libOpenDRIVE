#pragma once
#include "Lanes.h"

#include <cstdint>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

namespace odr
{

struct RoutingGraphEdge
{
    std::shared_ptr<Lane> start = nullptr;
    std::shared_ptr<Lane> end = nullptr;
};

class RoutingGraph
{
public:
    RoutingGraph() = default;
    virtual ~RoutingGraph() = default;

    void add_edge(const RoutingGraphEdge& edge);

    std::vector<RoutingGraphEdge> edges;

    std::unordered_map<std::shared_ptr<Lane>, std::set<std::shared_ptr<Lane>>> successors;
    std::unordered_map<std::shared_ptr<Lane>, std::set<std::shared_ptr<Lane>>> predecessors;
};

} // namespace odr
