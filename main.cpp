#include "Lanes.h"
#include "OpenDriveMap.h"
#include "Road.h"
#include "Utils.hpp"

#include <fstream>
#include <iostream>
#include <memory>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "ERROR: too few arguments" << std::endl;
        return -1;
    }
    odr::OpenDriveMap odr(argv[1]);

    std::vector<odr::Vec3D> pts;
    for (std::shared_ptr<odr::Road> road : odr.get_roads())
    {
        std::cout << "road: " << road->id << std::endl;
        odr::LaneSectionSet lanesections = road->get_lanesections();
        for (auto lanesec_iter = lanesections.begin(); lanesec_iter != lanesections.end(); lanesec_iter++)
        {
            const double lane_sec_len = (lanesec_iter == std::prev(lanesections.end())) ? road->length - (*lanesec_iter)->s0
                                                                                        : (*std::next(lanesec_iter))->s0 - (*lanesec_iter)->s0;
            for (std::shared_ptr<odr::Lane> lane : (*lanesec_iter)->get_lanes())
            {
                for (double s = (*lanesec_iter)->s0; s < (*lanesec_iter)->s0 + lane_sec_len; s += 0.1)
                {
                    pts.push_back(lane->get_outer_border_pt(s));
                    pts.push_back(road->get_xyz(s, 0, 0));
                }
            }
        }
    }
    std::cout << "Finished\n";

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
