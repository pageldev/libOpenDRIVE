#include "RoadMark.h"

namespace odr
{
// std::vector<Mesh3D> RoadMarkLines::generate_meshes() const
// {
//     std::vector<Mesh3D> meshes;
//     for (const Line3D& line : this->lines)
//     {
//         if (line.size() < 2)
//             continue;

//         Line3D inner_border_line, outer_border_line;

//         const double w2 = this->width * 0.5;
//         for (auto pt_iter = line.begin(); pt_iter != std::prev(line.end()); pt_iter++)
//         {
//             const Vec3D& cur_pt = *pt_iter;
//             const Vec3D& next_pt = *std::next(pt_iter);
//             if (cur_pt == next_pt)
//                 continue;

//             const Vec3D e_xyz = normalize(Vec3D{next_pt[0] - cur_pt[0], next_pt[1] - cur_pt[1], next_pt[2] - cur_pt[2]});

//             const Vec3D pt_inner_brdr = {cur_pt[0] + e_xyz[1] * w2, cur_pt[1] - e_xyz[0] * w2, cur_pt[2] + e_xyz[2] * w2};
//             inner_border_line.push_back(pt_inner_brdr);
//             const Vec3D pt_outer_brdr = {cur_pt[0] - e_xyz[1] * w2, cur_pt[1] + e_xyz[0] * w2, cur_pt[2] + e_xyz[2] * w2};
//             outer_border_line.push_back(pt_outer_brdr);

//             if (pt_iter == std::prev(line.end(), 2))
//             {
//                 const Vec3D last_pt_inner_brdr = {next_pt[0] + e_xyz[1] * w2, next_pt[1] - e_xyz[0] * w2, next_pt[2] + e_xyz[2] * w2};
//                 inner_border_line.push_back(last_pt_inner_brdr);
//                 const Vec3D last_pt_outer_brdr = {next_pt[0] - e_xyz[1] * w2, next_pt[1] + e_xyz[0] * w2, next_pt[2] + e_xyz[2] * w2};
//                 outer_border_line.push_back(last_pt_outer_brdr);
//             }
//         }

//         meshes.push_back(generate_mesh_from_borders(inner_border_line, outer_border_line));
//     }

//     return meshes;
// }

} // namespace odr