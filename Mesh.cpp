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
        size_t const* idxs = &(indices[idx]);
        ss_obj << "f " << idxs[0] << "//" << idxs[0] << ' ' << idxs[1] << "//" << idxs[1] << ' ' << idxs[2] << "//" << idxs[2] << std::endl;
    }

    return ss_obj.str();
}
} // namespace odr