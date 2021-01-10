#include "Lanes.h"
#include "Utils.hpp"

namespace odr
{
Lane::Lane(int id, bool level, std::string type) : id(id), level(level), type(type) {}

Mesh3D LaneLines::generate_mesh() const { return generate_mesh_from_borders(this->innner_border, this->outer_border); }

} // namespace odr