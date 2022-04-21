#include "ViewerUtils.h"
#include "Math.hpp"
#include "OpenDriveMap.h"
#include "RefLine.h"
#include "Road.h"

#include <vector>

namespace odr
{

Mesh3D get_refline_segments(const OpenDriveMap& odr_map, double eps)
{
    /* indices are pairs of vertices representing line segments */
    Mesh3D reflines;
    for (const auto& id_road : odr_map.id_to_road)
    {
        const Road&       road = id_road.second;
        const std::size_t idx_offset = reflines.vertices.size();

        std::set<double> s_vals = road.ref_line.approximate_linear(eps, 0.0, road.length);
        for (const double& s : s_vals)
        {
            reflines.vertices.push_back(road.ref_line.get_xyz(s));
            reflines.normals.push_back(normalize(road.ref_line.get_grad(s)));
        }

        for (std::size_t idx = idx_offset; idx < (idx_offset + s_vals.size() - 1); idx++)
        {
            reflines.indices.push_back(idx);
            reflines.indices.push_back(idx + 1);
        }
    }

    return reflines;
}

RoadNetworkMesh get_road_network_mesh(const OpenDriveMap& odr_map, double eps)
{
    RoadNetworkMesh  out_mesh;
    LanesMesh&       lanes_mesh = out_mesh.lanes_mesh;
    RoadmarksMesh&   roadmarks_mesh = out_mesh.roadmarks_mesh;
    RoadObjectsMesh& road_objects_mesh = out_mesh.road_objects_mesh;

    for (const auto& id_road : odr_map.id_to_road)
    {
        const Road& road = id_road.second;
        lanes_mesh.road_start_indices[lanes_mesh.vertices.size()] = road.id;
        roadmarks_mesh.road_start_indices[roadmarks_mesh.vertices.size()] = road.id;
        road_objects_mesh.road_start_indices[road_objects_mesh.vertices.size()] = road.id;

        for (const auto& s_lanesec : road.s_to_lanesection)
        {
            const LaneSection& lanesec = s_lanesec.second;
            lanes_mesh.lanesec_start_indices[lanes_mesh.vertices.size()] = lanesec.s0;
            roadmarks_mesh.lanesec_start_indices[roadmarks_mesh.vertices.size()] = lanesec.s0;
            for (const auto& id_lane : lanesec.id_to_lane)
            {
                const Lane&       lane = id_lane.second;
                const std::size_t lanes_idx_offset = lanes_mesh.vertices.size();
                lanes_mesh.lane_start_indices[lanes_idx_offset] = lane.id;
                lanes_mesh.add_mesh(road.get_lane_mesh(lane, eps));

                std::size_t roadmarks_idx_offset = roadmarks_mesh.vertices.size();
                roadmarks_mesh.lane_start_indices[roadmarks_idx_offset] = lane.id;
                const std::vector<RoadMark> roadmarks = lane.get_roadmarks(lanesec.s0, road.get_lanesection_end(lanesec));
                for (const RoadMark& roadmark : roadmarks)
                {
                    roadmarks_idx_offset = roadmarks_mesh.vertices.size();
                    roadmarks_mesh.roadmark_type_start_indices[roadmarks_idx_offset] = roadmark.type;
                    roadmarks_mesh.add_mesh(road.get_roadmark_mesh(lane, roadmark, eps));
                }
            }
        }

        for (const auto& id_road_object : road.id_to_object)
        {
            const RoadObject& road_object = id_road_object.second;
            const std::size_t road_objs_idx_offset = road_objects_mesh.vertices.size();
            road_objects_mesh.road_object_start_indices[road_objs_idx_offset] = road_object.id;
            road_objects_mesh.add_mesh(road.get_road_object_mesh(road_object, eps));
        }
    }

    return out_mesh;
}

} // namespace odr