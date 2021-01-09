#include "Lanes.h"

namespace odr
{
Lane::Lane(int id, bool level, std::string type) : id(id), level(level), type(type) {}

std::vector<Vec3D> Lane::get_road_mark_polygons() const
{
    std::vector<Vec3D> polygons;    
    return polygons;
}

} // namespace odr