#pragma once

#include "Geometries.h"
#include "Lanes.h"
#include "Road.h"

#include "pugixml.hpp"
#include "json/json.h"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <vector>


class OpenDriveMap
{
    public:
        OpenDriveMap(std::string xodr_file);
        void export_as_json(std::string out_file, double resolution = 0.1);

        std::string xodr_file;
        std::vector<std::shared_ptr<Road>> roads;
};