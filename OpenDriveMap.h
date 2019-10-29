#pragma once

#include "Road.h"

#include <memory>
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