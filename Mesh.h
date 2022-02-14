#pragma once

#include "Math.hpp"

#include <string>
#include <vector>

namespace odr
{
struct Mesh3D
{
    Mesh3D() = default;
    virtual ~Mesh3D() = default;

    void        add_mesh(const Mesh3D& other);
    std::string get_obj() const;

    std::vector<Vec3D>    vertices;
    std::vector<uint32_t> indices;

    std::vector<Vec3D> normals;
    std::vector<Vec2D> st_coordinates;
};
} // namespace odr