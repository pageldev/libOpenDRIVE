#include "Mesh.h"

#include <sstream>

namespace odr
{
std::string Mesh3D::get_obj() const
{
    std::stringstream ss_obj;
    for (const Vec3D& vt : this->vertices)
    {
        ss_obj << "v " << vt[0] << ' ' << vt[1] << ' ' << vt[2] << std::endl;
    }
    for (const Vec3D& vn : this->normals)
    {
        ss_obj << "vn " << vn[0] << ' ' << vn[1] << ' ' << vn[2] << std::endl;
    }
    for (size_t idx = 0; idx < this->indices.size(); idx += 3)
    {
        const size_t i1 = indices.at(idx) + 1;
        const size_t i2 = indices.at(idx + 1) + 1;
        const size_t i3 = indices.at(idx + 2) + 1;
        ss_obj << "f " << i1 << "//" << i1 << ' ' << i2 << "//" << i2 << ' ' << i3 << "//" << i3 << std::endl;
    }

    return ss_obj.str();
}
} // namespace odr