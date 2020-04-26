#include "OpenDriveMap.h"
#include "Road.h"
#include "Utils.h"

#include <iostream>
#include <memory>

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "ERROR: too few arguments" << std::endl;
        return -1;
    }
    OpenDriveMap odr(argv[1]);
    std::cout << odr.dump_json(0.1) << '\n';

    return 0;
}
