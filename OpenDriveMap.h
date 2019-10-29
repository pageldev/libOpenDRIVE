#pragma once

#include "Geometries.h"
#include "Lanes.h"
#include "Road.h"
#include "Utils.h"

#include "pugixml.hpp"
#include "json/json.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>


class OpenDriveMap
{
    public:
        OpenDriveMap(std::string xodr_file);
        void export_as_json(std::string out_file, double resolution = 0.01);

        std::string xodr_file;
        std::vector<std::shared_ptr<Road>> roads;
};