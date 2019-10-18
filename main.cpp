#include "OpenDriveMap.h"
#include "Road.h"
#include "Utils.h"

#include <iostream>
#include <memory>


int main(int argc, char**argv) {
    if( argc < 2 ) {
        std::cout << "ERROR: too few arguments" << std::endl;
        return -1;
    }
    OpenDriveMap odr(argv[1]);
    odr.export_as_json("out.json", 0.1);

    return 0;
}
