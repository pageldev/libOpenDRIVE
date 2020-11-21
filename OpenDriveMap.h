#pragma once

#include "Road.h"

#include <map>
#include <memory>
#include <string>

namespace odr
{
class OpenDriveMap
{
public:
    OpenDriveMap(std::string xodr_file);
    RoadSet get_roads();

    std::string xodr_file;
    std::string proj4;

    std::map<int, std::shared_ptr<Road>> roads;
};

} // namespace odr