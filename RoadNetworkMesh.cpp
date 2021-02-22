#include "RoadNetworkMesh.h"

#include <algorithm>

namespace odr
{
template<typename T>
std::vector<size_t> get_outline_indices(const std::map<size_t, T>& start_indices, const size_t num_vertices)
{
    std::vector<size_t> out_indices;
    for (auto idx_val_iter = start_indices.begin(); idx_val_iter != start_indices.end(); idx_val_iter++)
    {
        const size_t start_idx = idx_val_iter->first;
        const size_t end_idx = (std::next(idx_val_iter) == start_indices.end()) ? num_vertices : std::next(idx_val_iter)->first;
        for (size_t idx = start_idx; idx < end_idx - 2; idx += 2)
        {
            out_indices.push_back(idx);
            out_indices.push_back(idx + 2);
        }
        for (size_t idx = start_idx + 1; idx < end_idx - 2; idx += 2)
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

std::string MeshUnion::get_road_id(size_t vert_idx) const { return get_nearest_val<size_t, std::string>(this->road_start_indices, vert_idx); }

double MeshUnion::get_lanesec_s0(size_t vert_idx) const { return get_nearest_val<size_t, double>(this->lanesec_start_indices, vert_idx); }

int MeshUnion::get_lane_id(size_t vert_idx) const { return get_nearest_val<size_t, int>(this->lane_start_indices, vert_idx); }

std::array<size_t, 2> MeshUnion::get_idx_interval_road(size_t vert_idx) const
{
    return get_key_interval<size_t, std::string>(this->road_start_indices, vert_idx, this->vertices.size());
}

std::array<size_t, 2> MeshUnion::get_idx_interval_lanesec(size_t vert_idx) const
{
    return get_key_interval<size_t, double>(this->lanesec_start_indices, vert_idx, this->vertices.size());
}

std::array<size_t, 2> MeshUnion::get_idx_interval_lane(size_t vert_idx) const
{
    return get_key_interval<size_t, int>(this->lane_start_indices, vert_idx, this->vertices.size());
}

std::vector<size_t> LaneMeshUnion::get_lane_outline_indices() const
{
    return get_outline_indices<int>(this->lane_start_indices, this->vertices.size());
}

std::string RoadmarkMeshUnion::get_roadmark_type(size_t vert_idx) const
{
    return get_nearest_val<size_t, std::string>(this->roadmark_type_start_indices, vert_idx);
}

std::array<size_t, 2> RoadmarkMeshUnion::get_idx_interval_roadmark(size_t vert_idx) const
{
    return get_key_interval<size_t, std::string>(this->roadmark_type_start_indices, vert_idx, this->vertices.size());
}

std::vector<size_t> RoadmarkMeshUnion::get_roadmark_outline_indices() const
{
    return get_outline_indices<std::string>(this->roadmark_type_start_indices, this->vertices.size());
}

} // namespace odr