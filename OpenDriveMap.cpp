#include "OpenDriveMap.h"

OpenDriveMap::OpenDriveMap(std::string xodr_file) 
    : xodr_file(xodr_file)
{
    std::cout << this->xodr_file << std::endl;
    pugi::xml_document doc;
}