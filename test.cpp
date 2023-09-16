#include "Lane.h"
#include "LaneSection.h"
#include "Math.hpp"
#include "Mesh.h"
#include "OpenDriveMap.h"
#include "Road.h"

#include <fstream>
#include <iostream>
#include <stdio.h>
#include <vector>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        printf("ERROR: too few arguments\n");
        return -1;
    }
    odr::OpenDriveMap odr_map(argv[1]);
    const double      eps = 0.1;

    std::vector<odr::Vec3D> lane_pts;
    std::vector<odr::Vec3D> roadmark_pts;
    std::vector<odr::Vec3D> road_object_pts;
    std::vector<odr::Vec3D> road_signal_pts;

    for (odr::Road road : odr_map.get_roads())
    {
        printf("road: %s, length: %.2f\n", road.id.c_str(), road.length);
        for (odr::LaneSection lanesection : road.get_lanesections())
        {
            const double s_start = lanesection.s0;
            const double s_end = road.get_lanesection_end(lanesection);

            for (odr::Lane lane : lanesection.get_lanes())
            {
                auto lane_mesh = road.get_lane_mesh(lane, eps);
                lane_pts.insert(lane_pts.end(), lane_mesh.vertices.begin(), lane_mesh.vertices.end());

                auto roadmarks = lane.get_roadmarks(s_start, s_end);
                for (const auto& roadmark : roadmarks)
                {
                    auto roadmark_mesh = road.get_roadmark_mesh(lane, roadmark, eps);
                    roadmark_pts.insert(roadmark_pts.end(), roadmark_mesh.vertices.begin(), roadmark_mesh.vertices.end());
                }
            }
        }

        for (odr::RoadObject road_object : road.get_road_objects())
        {
            auto road_object_mesh = road.get_road_object_mesh(road_object, eps);
            road_object_pts.insert(road_object_pts.end(), road_object_mesh.vertices.begin(), road_object_mesh.vertices.end());
        }

        for (odr::RoadSignal road_signal : road.get_road_signals())
        {
            auto road_signal_mesh = road.get_road_signal_mesh(road_signal);
            road_signal_pts.insert(road_signal_pts.end(), road_signal_mesh.vertices.begin(), road_signal_mesh.vertices.end());
        }
    }

    printf("Finished, got %lu lane points, %lu roadmark points, %lu road object points, %lu road signal points\n",
           lane_pts.size(),
           roadmark_pts.size(),
           road_object_pts.size(),
           road_signal_pts.size());

    odr::RoadNetworkMesh road_network_mesh = odr_map.get_road_network_mesh(eps);
    printf("Got road network mesh\n");

    // std::ofstream out_file("out.obj");
    // out_file << road_network_mesh.get_mesh().get_obj() << std::endl;
    // out_file.close();
    // printf("Wrote .obj file to 'out.obj'\n");

    return 0;
}
