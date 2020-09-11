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
    OpenDriveMap(const std::string xodr_file);

    const std::string                    xodr_file;
    std::map<int, std::shared_ptr<Road>> roads;
};

} // namespace odr