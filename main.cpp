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
        printf("road: %s\n", road->id.c_str());
        for (std::shared_ptr<odr::LaneSection> lanesec : road->get_lanesections())
        {
            std::vector<odr::LaneLines> lane_lines = lanesec->get_lane_lines(0.1);
            for (const auto& lane_line : lane_lines)
                pts.insert(pts.end(), lane_line.outer_border.begin(), lane_line.outer_border.end());
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
