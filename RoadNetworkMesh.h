#pragma once

#include "Utils.hpp"

#include <map>
#include <string>
#include <vector>

namespace odr
{
struct RoadNetworkMesh : public Mesh3D
{
    std::string get_road_id(size_t vert_idx) const;
    double      get_lanesec_s0(size_t vert_idx) const;
    int         get_lane_id(size_t vert_idx) const;

    std::vector<size_t> get_lane_outline_indices() const;

    std::map<size_t, std::string> road_start_indices;
    std::map<size_t, double>      lanesec_start_indices;
    std::map<size_t, int>         lane_start_indices;
};

} // namespace odr