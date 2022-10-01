#pragma once
#include "Lane.h"

#include <cstddef>
#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace odr
{

struct RoutingGraphEdge
{
    RoutingGraphEdge(LaneKey from, LaneKey to, double length);

    LaneKey from;
    LaneKey to;
    double  weight = 0;
};

struct WeightedLaneKey : public LaneKey
{
    WeightedLaneKey(const LaneKey& lane_key, double weight);
    WeightedLaneKey(std::string road_id, double lanesection_s0, int lane_id, double weight);

    double weight = 0;
};

} // namespace odr

namespace std
{
template<>
struct hash<odr::RoutingGraphEdge>
{
    size_t operator()(const odr::RoutingGraphEdge& e) const
    {
        // shift bits to prevent xor canceling out same values, e.g. hash("a","a",1) == hash("b","b",1)
        return ((hash<odr::LaneKey>()(e.from) ^ (hash<odr::LaneKey>()(e.to) << 1)) >> 1) ^ (hash<double>()(e.weight) << 1);
    }
};

template<>
struct equal_to<odr::RoutingGraphEdge>
{
    size_t operator()(const odr::RoutingGraphEdge& lhs, const odr::RoutingGraphEdge& rhs) const
    {
        return equal_to<odr::LaneKey>{}(lhs.from, rhs.from) && equal_to<odr::LaneKey>{}(lhs.to, rhs.to) && equal_to<double>{}(lhs.weight, rhs.weight);
    }
};

template<>
struct hash<odr::WeightedLaneKey>
{
    size_t operator()(const odr::WeightedLaneKey& w_key) const { return (hash<odr::LaneKey>()(w_key) ^ (hash<double>()(w_key.weight) << 1)); }
};

template<>
struct equal_to<odr::WeightedLaneKey>
{
    size_t operator()(const odr::WeightedLaneKey& lhs, const odr::WeightedLaneKey& rhs) const
    {
        return equal_to<odr::LaneKey>{}(lhs, rhs) && equal_to<double>{}(lhs.weight, rhs.weight);
    }
};
} // namespace std

namespace odr
{

class RoutingGraph
{
public:
    RoutingGraph() = default;
    void add_edge(const RoutingGraphEdge& edge);

    std::vector<LaneKey> get_lane_successors(const LaneKey& lane_key) const;
    std::vector<LaneKey> get_lane_predecessors(const LaneKey& lane_key) const;
    std::vector<LaneKey> shortest_path(const LaneKey& from, const LaneKey& to) const;

    std::unordered_set<RoutingGraphEdge>                             edges;
    std::unordered_map<LaneKey, std::unordered_set<WeightedLaneKey>> lane_key_to_successors;
    std::unordered_map<LaneKey, std::unordered_set<WeightedLaneKey>> lane_key_to_predecessors;
};

} // namespace odr
