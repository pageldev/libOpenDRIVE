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
    std::string dump_json(const double resolution = 0.01) const;

    const std::string                    xodr_file;
    std::map<int, std::shared_ptr<Road>> roads;
};

} // namespace odr