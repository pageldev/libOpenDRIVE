#pragma once

#include "Geometries.h"
#include "Lanes.h"
#include "Road.h"

#include "pugixml.hpp"

#include <algorithm>
#include <iostream>
#include <string>
#include <memory>
#include <vector>


class OpenDriveMap
{
    public:
        OpenDriveMap(std::string xodr_file);

        std::string xodr_file;
        std::vector<std::shared_ptr<Road>> roads;
};