#include "RoutingGraph.h"

#include <algorithm>
#include <limits>
#include <queue>
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
    if (from == to)
    {
        return {from};
    }

    // Priority queue to maintain open set
    std::priority_queue<WeightedLaneKey, std::vector<WeightedLaneKey>, std::greater<odr::WeightedLaneKey>> open_set;
    // Maps to store costs and previous nodes
    std::unordered_map<LaneKey, double>  cost_from_start;
    std::unordered_map<LaneKey, LaneKey> came_from;

    // Initialize the start node
    cost_from_start.emplace(from, 0.0);
    open_set.emplace(from, 0.0);

    while (!open_set.empty())
    {
        // Get the current node with the smallest weight
        WeightedLaneKey current_weighted = open_set.top();
        LaneKey         current = current_weighted;
        open_set.pop();

        // Skip stale entries
        auto current_cost_itr = cost_from_start.find(current);
        if (current_cost_itr == cost_from_start.end() || current_weighted.weight > current_cost_itr->second)
        {
            continue;
        }

        // If the goal is reached, reconstruct the path
        if (current == to)
        {
            std::vector<LaneKey> path;
            LaneKey              at = to;
            while (true)
            {
                path.push_back(at);
                auto it = came_from.find(at);
                if (it == came_from.end())
                {
                    break;
                }
                at = it->second;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Get successors of the current node
        auto succ_itr = lane_key_to_successors.find(current);
        if (succ_itr == lane_key_to_successors.end())
        {
            continue;
        }

        for (const auto& weighted_successor : succ_itr->second)
        {
            LaneKey neighbor = weighted_successor;
            double  weight = weighted_successor.weight;

            // Check the current cost of the neighbor
            auto   neighbor_cost_itr = cost_from_start.find(neighbor);
            double current_cost = (neighbor_cost_itr != cost_from_start.end()) ? neighbor_cost_itr->second : std::numeric_limits<double>::max();

            // Compute the alternative path cost
            double alternative_path_cost = current_cost_itr->second + weight;
            if (alternative_path_cost < current_cost)
            {
                // Update cost_from_start
                if (neighbor_cost_itr == cost_from_start.end())
                {
                    cost_from_start.emplace(neighbor, alternative_path_cost);
                }
                else
                {
                    neighbor_cost_itr->second = alternative_path_cost;
                }

                // Update came_from
                auto came_from_itr = came_from.find(neighbor);
                if (came_from_itr == came_from.end())
                {
                    came_from.emplace(neighbor, current);
                }
                else
                {
                    came_from_itr->second = current;
                }

                // Add the neighbor to the open set
                open_set.emplace(neighbor, alternative_path_cost);
            }
        }
    }

    return {}; // No path found
}

} // namespace odr