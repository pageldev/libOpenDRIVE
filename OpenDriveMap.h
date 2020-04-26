#pragma once

#include "Road.h"

#include <map>
#include <memory>
#include <string>

class OpenDriveMap
{
public:
    OpenDriveMap(std::string xodr_file);
    std::string dump_json(double resolution = 0.01);

    std::string xodr_file;
    std::map<int, std::shared_ptr<Road>> roads;
};