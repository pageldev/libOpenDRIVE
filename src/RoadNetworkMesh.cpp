#include "RoadNetworkMesh.h"
#include "Utils.hpp"

namespace odr
{
template<typename T>
std::vector<size_t> get_outline_indices(const std::map<size_t, T>& start_indices, const std::size_t num_vertices)
{
    std::vector<size_t> out_indices;
    for (auto idx_val_iter = start_indices.begin(); idx_val_iter != start_indices.end(); idx_val_iter++)
    {
        const std::size_t start_idx = idx_val_iter->first;
        const std::size_t end_idx = (std::next(idx_val_iter) == start_indices.end()) ? num_vertices : std::next(idx_val_iter)->first;
        for (std::size_t idx = start_idx; idx < end_idx - 2; idx += 2)
        {
            out_indices.push_back(idx);
            out_indices.push_back(idx + 2);
        }
        for (std::size_t idx = start_idx + 1; idx < end_idx - 2; idx += 2)
        {
            out_indices.push_back(idx);
            out_indices.push_back(idx + 2);
        }
        out_indices.push_back(start_idx);
        out_indices.push_back(start_idx + 1);
        out_indices.push_back(end_idx - 2);
        out_indices.push_back(end_idx - 1);
    }

    return out_indices;
}

std::string RoadsMesh::get_road_id(const std::size_t vert_idx) const
{
    return get_nearest_lower_val<size_t, std::string>(this->road_start_indices, vert_idx);
}

double LanesMesh::get_lanesec_s0(const std::size_t vert_idx) const
{
    return get_nearest_lower_val<size_t, double>(this->lanesec_start_indices, vert_idx);
}

int LanesMesh::get_lane_id(const std::size_t vert_idx) const { return get_nearest_lower_val<size_t, int>(this->lane_start_indices, vert_idx); }

std::array<size_t, 2> RoadsMesh::get_idx_interval_road(const std::size_t vert_idx) const
{
    return get_key_interval<size_t, std::string>(this->road_start_indices, vert_idx, this->vertices.size());
}

std::array<size_t, 2> LanesMesh::get_idx_interval_lanesec(const std::size_t vert_idx) const
{
    return get_key_interval<size_t, double>(this->lanesec_start_indices, vert_idx, this->vertices.size());
}

std::array<size_t, 2> LanesMesh::get_idx_interval_lane(const std::size_t vert_idx) const
{
    return get_key_interval<size_t, int>(this->lane_start_indices, vert_idx, this->vertices.size());
}

std::vector<size_t> LanesMesh::get_lane_outline_indices() const { return get_outline_indices<int>(this->lane_start_indices, this->vertices.size()); }

std::string RoadmarksMesh::get_roadmark_type(const std::size_t vert_idx) const
{
    return get_nearest_lower_val<size_t, std::string>(this->roadmark_type_start_indices, vert_idx);
}

std::array<size_t, 2> RoadmarksMesh::get_idx_interval_roadmark(const std::size_t vert_idx) const
{
    return get_key_interval<size_t, std::string>(this->roadmark_type_start_indices, vert_idx, this->vertices.size());
}

std::vector<size_t> RoadmarksMesh::get_roadmark_outline_indices() const
{
    return get_outline_indices<std::string>(this->roadmark_type_start_indices, this->vertices.size());
}

std::string RoadObjectsMesh::get_road_object_id(const std::size_t vert_idx) const
{
    return get_nearest_lower_val<size_t, std::string>(this->road_object_start_indices, vert_idx);
}

std::array<size_t, 2> RoadObjectsMesh::get_idx_interval_road_object(const std::size_t vert_idx) const
{
    return get_key_interval<size_t, std::string>(this->road_object_start_indices, vert_idx, this->vertices.size());
}

std::string RoadSignalsMesh::get_road_signal_id(const std::size_t vert_idx) const
{
    return get_nearest_lower_val<size_t, std::string>(this->road_signal_start_indices, vert_idx);
}

std::array<size_t, 2> RoadSignalsMesh::get_idx_interval_signal(const std::size_t vert_idx) const
{
    return get_key_interval<size_t, std::string>(this->road_signal_start_indices, vert_idx, this->vertices.size());
}

Mesh3D RoadNetworkMesh::get_mesh() const
{
    Mesh3D out_mesh;
    out_mesh.add_mesh(this->lanes_mesh);
    out_mesh.add_mesh(this->roadmarks_mesh);
    out_mesh.add_mesh(this->road_objects_mesh);
    out_mesh.add_mesh(this->road_signals_mesh);
    return out_mesh;
}

} // namespace odr