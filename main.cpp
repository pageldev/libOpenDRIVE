#include "OpenDriveMap.h"
#include "Road.h"

#include <iostream>
#include <memory>


int main(int argc, char**argv) {
    if( argc < 2 ) {
        std::cout << "ERROR: too few arguments" << std::endl;
        return -1;
    }
    OpenDriveMap odr(argv[1]);
    odr.export_as_geojson("out.json");
    odr.export_as_obj("out.obj");
    return 0;
}
