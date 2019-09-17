#include "OpenDriveMap.h"

#include <iostream>

int main(int argc, char**argv) {
    if( argc < 2 ) {
        std::cout << "ERROR: too few arguments" << std::endl;
        return -1;
    }
    OpenDriveMap odr(argv[1]);
    return 0;
}
