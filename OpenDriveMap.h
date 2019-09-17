#include "pugixml.hpp"

#include <iostream>
#include <string>

class OpenDriveMap
{
    public:
        OpenDriveMap(std::string xodr_file);

    private:
        std::string xodr_file;
};