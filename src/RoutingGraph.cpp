#include "RoutingGraph.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <queue>
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
    std::vector<LaneKey>                successor_lane_keys(res.begin(), res.end());
    return successor_lane_keys;
}

std::vector<LaneKey> RoutingGraph::get_lane_predecessors(const LaneKey& lane_key) const
{
    std::unordered_set<WeightedLaneKey> res = try_get_val(this->lane_key_to_predecessors, lane_key, std::unordered_set<WeightedLaneKey>{});
    std::vector<LaneKey>                predecessor_lane_keys(res.begin(), res.end());
    return predecessor_lane_keys;
}
std::vector<LaneKey> RoutingGraph::shortest_path(const LaneKey& from, const LaneKey& to) const
{
    // Using a min-heap to represent the open set
    std::priority_queue<WeightedLaneKey, std::vector<WeightedLaneKey>, CompareLaneKey> open_set;

    // Maps to keep track of the best cost from start to a node and the best previous node to reach a node
    std::unordered_map<LaneKey, double>  cost_from_start;
    std::unordered_map<LaneKey, LaneKey> came_from;

    // Initialize the start node in the open set with zero cost
    cost_from_start[from] = 0.0;
    open_set.push(WeightedLaneKey(from, 0.0));

    while (!open_set.empty())
    {
        WeightedLaneKey current_weighted = open_set.top();
        LaneKey         current = current_weighted;
        open_set.pop();

        // If the goal is reached, stop and construct the path
        if (current == to)
        {
            std::vector<LaneKey> path;
            for (LaneKey at = to; at != from; at = came_from[at])
            {
                path.push_back(at);
            }
            path.push_back(from);
            std::reverse(path.begin(), path.end());
            return path;
        }
        // Check if this is a stale entry
        if (current_weighted.weight > cost_from_start[current])
        {
            continue; // Skip processing this node because we have found a better path
        }
        // Explore the neighbors
        for (const auto& weighted_successor : this->lane_key_to_successors.at(current))
        {
            LaneKey neighbor = weighted_successor;
            double  weight = weighted_successor.weight;
            double  alternative_path_cost = cost_from_start[current] + weight;

            // If new cost is better, or the neighbor is not in cost_from_start, update the cost and path
            if (cost_from_start.find(neighbor) == cost_from_start.end() || alternative_path_cost < cost_from_start[neighbor])
            {
                cost_from_start[neighbor] = alternative_path_cost;
                came_from[neighbor] = current;
                open_set.push(WeightedLaneKey(neighbor, alternative_path_cost));
            }
        }
    }

    // No path found
    return {};
}

} // namespace odr