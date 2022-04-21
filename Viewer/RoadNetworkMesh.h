#pragma once
#include "Mesh.h"

#include <array>
#include <map>
#include <string>
#include <vector>

namespace odr
{
struct RoadsMesh : public Mesh3D
{
    RoadsMesh() = default;
    virtual ~RoadsMesh() = default;

    std::string get_road_id(std::size_t vert_idx) const;

    std::array<size_t, 2> get_idx_interval_road(std::size_t vert_idx) const;

    std::map<size_t, std::string> road_start_indices; // start idx of road can be used as id
};

struct LanesMesh : public RoadsMesh
{
    LanesMesh() = default;
    virtual ~LanesMesh() = default;

    double get_lanesec_s0(std::size_t vert_idx) const;
    int    get_lane_id(std::size_t vert_idx) const;

    std::array<size_t, 2> get_idx_interval_lanesec(std::size_t vert_idx) const;
    std::array<size_t, 2> get_idx_interval_lane(std::size_t vert_idx) const;

    std::vector<size_t> get_lane_outline_indices() const;

    std::map<size_t, double> lanesec_start_indices;
    std::map<size_t, int>    lane_start_indices;
};

struct RoadmarksMesh : public LanesMesh
{
    RoadmarksMesh() = default;
    virtual ~RoadmarksMesh() = default;

    std::string           get_roadmark_type(std::size_t vert_idx) const;
    std::array<size_t, 2> get_idx_interval_roadmark(std::size_t vert_idx) const;
    std::vector<size_t>   get_roadmark_outline_indices() const;

    std::map<size_t, std::string> roadmark_type_start_indices;
};

struct RoadObjectsMesh : public RoadsMesh
{
    std::string get_road_object_id(std::size_t vert_idx) const;

    std::array<size_t, 2> get_idx_interval_road_object(std::size_t vert_idx) const;

    std::map<size_t, std::string> road_object_start_indices;
};

struct RoadNetworkMesh
{
    Mesh3D get_mesh() const;

    LanesMesh       lanes_mesh;
    RoadmarksMesh   roadmarks_mesh;
    RoadObjectsMesh road_objects_mesh;
};

} // namespace odr