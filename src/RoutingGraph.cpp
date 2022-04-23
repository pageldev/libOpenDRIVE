#include "RoutingGraph.h"

#include <algorithm>
#include <limits>
#include <utility>

namespace odr
{

RoutingGraphEdge::RoutingGraphEdge(LaneKey from, LaneKey to, double weight) : from(from), to(to), weight(weight) {}

WeightedLaneKey::WeightedLaneKey(const LaneKey& lane_key, double weight) : LaneKey(lane_key), weight(weight) {}

WeightedLaneKey::WeightedLaneKey(std::string road_id, double lanesection_s0, int lane_id, double weight) :
    LaneKey(road_id, lanesection_s0, lane_id), weight(weight)
{
}

void RoutingGraph::add_edge(const RoutingGraphEdge& edge)
{
    this->edges.insert(edge);
    this->lane_key_to_successors[edge.from].insert(WeightedLaneKey(edge.to, edge.weight));
    this->lane_key_to_predecessors[edge.to].insert(WeightedLaneKey(edge.from, edge.weight));
}

std::vector<LaneKey> RoutingGraph::get_lane_successors(const LaneKey& lane_key) const
{
    std::unordered_set<WeightedLaneKey> res = try_get_val(this->lane_key_to_successors, lane_key, std::unordered_set<WeightedLaneKey>{});
    std::vector<LaneKey> successor_lane_keys(res.begin(), res.end());
    return successor_lane_keys;
}

std::vector<LaneKey> RoutingGraph::get_lane_predecessors(const LaneKey& lane_key) const
{
    std::unordered_set<WeightedLaneKey> res = try_get_val(this->lane_key_to_predecessors, lane_key, std::unordered_set<WeightedLaneKey>{});
    std::vector<LaneKey> predecessor_lane_keys(res.begin(), res.end());
    return predecessor_lane_keys;
}

std::vector<LaneKey> RoutingGraph::shortest_path(const LaneKey& start, const LaneKey& finish) const
{
    std::vector<LaneKey>                 nodes;
    std::vector<LaneKey>                 path;
    std::unordered_map<LaneKey, double>  weights;
    std::unordered_map<LaneKey, LaneKey> previous;

    auto comparator = [&](const LaneKey& lhs, const LaneKey& rhs) { return weights[lhs] > weights[rhs]; };

    for (const auto& lane_key_successors : this->lane_key_to_successors)
    {
        const LaneKey& lane_key = lane_key_successors.first;
        if (std::equal_to<odr::LaneKey>{}(lane_key, start))
            weights[lane_key] = 0;
        else
            weights[lane_key] = std::numeric_limits<double>::max();

        nodes.push_back(lane_key);
        std::push_heap(nodes.begin(), nodes.end(), comparator);
    }

    while (nodes.empty() == false)
    {
        std::pop_heap(nodes.begin(), nodes.end(), comparator);
        LaneKey smallest = nodes.back();
        nodes.pop_back();

        if (std::equal_to<LaneKey>{}(smallest, finish))
        {
            while (previous.find(smallest) != previous.end())
            {
                path.push_back(smallest);
                smallest = previous.at(smallest);
            }
            break;
        }

        if (weights.at(smallest) == std::numeric_limits<double>::max())
            break;

        for (const auto& successor : this->lane_key_to_successors.at(smallest))
        {
            const double alt = weights.at(smallest) + successor.weight;
            if (alt < weights.at(successor))
            {
                weights[successor] = alt;
                previous.insert({successor, smallest});
                std::make_heap(nodes.begin(), nodes.end(), comparator);
            }
        }
    }

    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace odr