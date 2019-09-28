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
        void export_as_obj(std::string out_file, double resolution = 0.1);
        void export_as_json(std::string out_file, double resolution = 0.1);

        std::string xodr_file;
        std::map<int /*id*/, std::shared_ptr<Road>> roads;
};