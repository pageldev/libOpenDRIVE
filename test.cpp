#include "Lanes.h"
#include "Math.hpp"
#include "OpenDriveMap.h"
#include "RefLine.h"
#include "Road.h"

#include <fstream>
#include <memory>
#include <stdio.h>
#include <vector>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        printf("ERROR: too few arguments\n");
        return -1;
    }
    odr::OpenDriveMap odr(argv[1]);

    std::vector<odr::Vec3D> pts;
    for (std::shared_ptr<odr::Road> road : odr.get_roads())
    {
        printf("road: %s, length: %.2f\n", road->id.c_str(), road->length);
        for (std::shared_ptr<odr::LaneSection> lanesec : road->get_lanesections())
        {
            for (std::shared_ptr<odr::Lane> lane : lanesec->get_lanes())
            {
                auto lane_mesh = lane->get_mesh(lanesec->s0, lanesec->get_end(), 0.1);
                pts.insert(pts.end(), lane_mesh.vertices.begin(), lane_mesh.vertices.end());
            }
        }
    }
    printf("Finished\n");

    if (argc == 3)
    {
        std::ofstream out_file;
        out_file.open(argv[2]);

        odr::RoadNetworkMesh road_network_mesh = odr.get_mesh(0.1);
        out_file << road_network_mesh.get_mesh().get_obj() << std::endl;

        out_file.close();
    }

    return 0;
}
