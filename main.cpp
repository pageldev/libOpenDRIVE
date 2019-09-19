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

    for( std::shared_ptr<Road> road : odr.roads ) {
        std::cout << road->length << std::endl;
    }

    return 0;
}
