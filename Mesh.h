#pragma once

#include "Math.hpp"

#include <string>
#include <vector>

namespace odr
{
struct Mesh3D
{
    std::string get_obj() const;

    std::vector<Vec3D>  vertices;
    std::vector<size_t> indices;

    std::vector<Vec3D> normals;
    std::vector<Vec2D> st_coordinates;
};
} // namespace odr