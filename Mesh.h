#pragma once

#include "Math.hpp"

#include <vector>

namespace odr
{
struct Mesh3D
{
    std::vector<Vec3D>  vertices;
    std::vector<size_t> indices;
    std::vector<Vec2D>  st_coordinates;
};
} // namespace odr