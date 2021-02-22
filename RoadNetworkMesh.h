#pragma once

#include "Utils.hpp"

#include <map>
#include <string>
#include <vector>

namespace odr
{
struct MeshUnion : public Mesh3D
{
    std::string get_road_id(size_t vert_idx) const;
    double      get_lanesec_s0(size_t vert_idx) const;
    int         get_lane_id(size_t vert_idx) const;

    std::array<size_t, 2> get_idx_interval_road(size_t vert_idx) const;
    std::array<size_t, 2> get_idx_interval_lanesec(size_t vert_idx) const;
    std::array<size_t, 2> get_idx_interval_lane(size_t vert_idx) const;

    std::map<size_t, std::string> road_start_indices;
    std::map<size_t, double>      lanesec_start_indices;
    std::map<size_t, int>         lane_start_indices;
};

struct LaneMeshUnion : public MeshUnion
{
    std::vector<size_t> get_lane_outline_indices() const;
};

struct RoadmarkMeshUnion : public MeshUnion
{
    std::string           get_roadmark_type(size_t vert_idx) const;
    std::array<size_t, 2> get_idx_interval_roadmark(size_t vert_idx) const;
    std::vector<size_t>   get_roadmark_outline_indices() const;

    std::map<size_t, std::string> roadmark_type_start_indices;
};

struct RoadNetworkMesh
{
    LaneMeshUnion     lane_mesh_union;
    RoadmarkMeshUnion roadmark_mesh_union;
};

} // namespace odr