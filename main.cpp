#include "OpenDriveMap.h"
#include "Road.h"
#include "Utils.hpp"

#include <iostream>
#include <memory>

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "ERROR: too few arguments" << std::endl;
        return -1;
    }
    odr::OpenDriveMap odr(argv[1]);
    odr.dump_json(0.1);
    std::cout << "Finished\n";

    return 0;
}
