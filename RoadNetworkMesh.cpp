#include "RoadNetworkMesh.h"

#include <algorithm>

namespace odr
{
std::string MeshUnion::get_road_id(size_t vert_idx) const
{
    auto idx_road_id_iter = this->road_start_indices.upper_bound(vert_idx);
    if (idx_road_id_iter != this->road_start_indices.begin())
        idx_road_id_iter--;
    return idx_road_id_iter->second;
}

double MeshUnion::get_lanesec_s0(size_t vert_idx) const
{
    auto idx_lanesec_s0_iter = this->lanesec_start_indices.upper_bound(vert_idx);
    if (idx_lanesec_s0_iter != this->lanesec_start_indices.begin())
        idx_lanesec_s0_iter--;
    return idx_lanesec_s0_iter->second;
}

int MeshUnion::get_lane_id(size_t vert_idx) const
{
    auto idx_lane_id_iter = this->lane_start_indices.upper_bound(vert_idx);
    if (idx_lane_id_iter != this->lane_start_indices.begin())
        idx_lane_id_iter--;
    return idx_lane_id_iter->second;
}

std::array<size_t, 2> MeshUnion::get_idx_interval_road(size_t vert_idx) const
{
    auto idx_road_id_iter = this->road_start_indices.upper_bound(vert_idx);
    if (idx_road_id_iter != this->road_start_indices.begin())
        idx_road_id_iter--;
    const size_t start_idx = idx_road_id_iter->first;
    const size_t end_idx =
        (std::next(idx_road_id_iter) == this->road_start_indices.end()) ? this->vertices.size() : std::next(idx_road_id_iter)->first;

    return {start_idx, end_idx};
}

std::array<size_t, 2> MeshUnion::get_idx_interval_lanesec(size_t vert_idx) const
{
    auto idx_lanesec_s0_iter = this->lanesec_start_indices.upper_bound(vert_idx);
    if (idx_lanesec_s0_iter != this->lanesec_start_indices.begin())
        idx_lanesec_s0_iter--;
    const size_t start_idx = idx_lanesec_s0_iter->first;
    const size_t end_idx =
        (std::next(idx_lanesec_s0_iter) == this->lanesec_start_indices.end()) ? this->vertices.size() : std::next(idx_lanesec_s0_iter)->first;

    return {start_idx, end_idx};
}

std::array<size_t, 2> MeshUnion::get_idx_interval_lane(size_t vert_idx) const
{
    auto idx_lane_id_iter = this->lane_start_indices.upper_bound(vert_idx);
    if (idx_lane_id_iter != this->lane_start_indices.begin())
        idx_lane_id_iter--;
    const size_t start_idx = idx_lane_id_iter->first;
    const size_t end_idx =
        (std::next(idx_lane_id_iter) == this->lane_start_indices.end()) ? this->vertices.size() : std::next(idx_lane_id_iter)->first;

    return {start_idx, end_idx};
}

std::vector<size_t> LaneMeshUnion::get_lane_outline_indices() const
{
    std::vector<size_t> out_indices;
    for (auto idx_id_iter = this->lane_start_indices.begin(); idx_id_iter != this->lane_start_indices.end(); idx_id_iter++)
    {
        const size_t start_idx = idx_id_iter->first;
        const size_t end_idx = (std::next(idx_id_iter) == this->lane_start_indices.end()) ? this->vertices.size() : std::next(idx_id_iter)->first;
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

} // namespace odr