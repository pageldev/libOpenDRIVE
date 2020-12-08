#include "Lanes.h"
#include "Math.hpp"
#include "OpenDriveMap.h"
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
        printf("road: %d\n", road->id);
        for (std::shared_ptr<odr::LaneSection> lanesec : road->get_lanesections())
        {
            std::vector<odr::LaneVertices> lane_vertices = lanesec->get_lane_vertices(0.1);
            for (const auto& lane_vertice : lane_vertices)
                pts.insert(pts.end(), lane_vertice.vertices.begin(), lane_vertice.vertices.end());
        }
    }
    printf("Finished\n");

    if (argc == 3)
    {
        std::ofstream out_file;
        out_file.open(argv[2]);
        for (const auto& pt : pts)
            out_file << pt[0] << ' ' << pt[1] << ' ' << pt[2] << std::endl;
        out_file.close();
    }

    return 0;
}
