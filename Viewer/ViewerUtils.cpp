#include "ViewerUtils.h"
#include "Math.hpp"
#include "RefLine.h"
#include "Road.h"

#include <memory>
#include <vector>

namespace odr
{

Mesh3D get_refline_segments(const OpenDriveMap& odr_map, double eps)
{
    /* indices are pairs of vertices representing line segments */
    Mesh3D reflines;
    for (std::shared_ptr<const Road> road : odr_map.get_roads())
    {
        const size_t idx_offset = reflines.vertices.size();

        std::set<double> s_vals = road->ref_line->approximate_linear(eps, 0.0, road->length);
        for (const double& s : s_vals)
        {
            reflines.vertices.push_back(road->ref_line->get_xyz(s));
            reflines.normals.push_back(normalize(road->ref_line->get_grad(s)));
        }

        for (size_t idx = idx_offset; idx < (idx_offset + s_vals.size() - 1); idx++)
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

    for (std::shared_ptr<const Road> road : odr_map.get_roads())
    {
        lanes_mesh.road_start_indices[lanes_mesh.vertices.size()] = road->id;
        roadmarks_mesh.road_start_indices[roadmarks_mesh.vertices.size()] = road->id;
        road_objects_mesh.road_start_indices[road_objects_mesh.vertices.size()] = road->id;

        for (std::shared_ptr<const LaneSection> lanesec : road->get_lanesections())
        {
            lanes_mesh.lanesec_start_indices[lanes_mesh.vertices.size()] = lanesec->s0;
            roadmarks_mesh.lanesec_start_indices[roadmarks_mesh.vertices.size()] = lanesec->s0;
            for (std::shared_ptr<const Lane> lane : lanesec->get_lanes())
            {
                const size_t lanes_idx_offset = lanes_mesh.vertices.size();
                lanes_mesh.lane_start_indices[lanes_idx_offset] = lane->id;
                lanes_mesh.add_mesh(lane->get_mesh(lanesec->s0, lanesec->get_end(), eps));

                size_t roadmarks_idx_offset = roadmarks_mesh.vertices.size();
                roadmarks_mesh.lane_start_indices[roadmarks_idx_offset] = lane->id;
                const std::vector<std::shared_ptr<RoadMark>> roadmarks = lane->get_roadmarks(lanesec->s0, lanesec->get_end());
                for (std::shared_ptr<const RoadMark> roadmark : roadmarks)
                {
                    roadmarks_idx_offset = roadmarks_mesh.vertices.size();
                    roadmarks_mesh.roadmark_type_start_indices[roadmarks_idx_offset] = roadmark->type;
                    roadmarks_mesh.add_mesh(lane->get_roadmark_mesh(roadmark, eps));
                }
            }
        }

        for (std::shared_ptr<const RoadObject> road_object : road->get_road_objects())
        {
            const size_t road_objs_idx_offset = road_objects_mesh.vertices.size();
            road_objects_mesh.road_object_start_indices[road_objs_idx_offset] = road_object->id;
            road_objects_mesh.add_mesh(road_object->get_mesh(eps));
        }
    }

    return out_mesh;
}

} // namespace odr