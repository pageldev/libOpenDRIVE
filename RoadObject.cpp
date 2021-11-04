#include "RoadObject.h"
#include "RefLine.h"
#include "Road.h"

#include <cmath>
#include <math.h>

namespace odr
{
Mesh3D RoadObject::get_cylinder(double eps, double radius, double height)
{
    Mesh3D cylinder_mesh;
    cylinder_mesh.vertices.push_back({0, 0, 0});
    cylinder_mesh.vertices.push_back({0, 0, height});

    eps = 0.5 * eps; // reduce eps a bit, cylinders more subsceptible to low resolution
    const double eps_angle = (radius <= eps) ? M_PI / 6 : std::acos((radius * radius - 4 * radius * eps + 2 * eps * eps) / (radius * radius));

    std::vector<double> angles;
    for (double alpha = 0; alpha < 2 * M_PI; alpha += eps_angle)
        angles.push_back(alpha);
    angles.push_back(2 * M_PI);

    for (const double& alpha : angles)
    {
        const Vec3D circle_pt_bottom = {radius * std::cos(alpha), radius * std::sin(alpha), 0};
        const Vec3D circle_pt_top = {radius * std::cos(alpha), radius * std::sin(alpha), height};
        cylinder_mesh.vertices.push_back(circle_pt_bottom);
        cylinder_mesh.vertices.push_back(circle_pt_top);

        if (cylinder_mesh.vertices.size() > 5)
        {
            const size_t          cur_idx = cylinder_mesh.vertices.size() - 1;
            std::array<size_t, 6> top_bottom_idx_patch = {0, cur_idx - 1, cur_idx - 3, 1, cur_idx - 2, cur_idx};
            cylinder_mesh.indices.insert(cylinder_mesh.indices.end(), top_bottom_idx_patch.begin(), top_bottom_idx_patch.end());
            std::array<size_t, 6> wall_idx_patch = {cur_idx, cur_idx - 2, cur_idx - 3, cur_idx, cur_idx - 3, cur_idx - 1};
            cylinder_mesh.indices.insert(cylinder_mesh.indices.end(), wall_idx_patch.begin(), wall_idx_patch.end());
        }
    }

    return cylinder_mesh;
}

Mesh3D RoadObject::get_box(double w, double l, double h)
{
    Mesh3D box_mesh;
    box_mesh.vertices = {Vec3D{l / 2, w / 2, 0},
                         Vec3D{-l / 2, w / 2, 0},
                         Vec3D{-l / 2, -w / 2, 0},
                         Vec3D{l / 2, -w / 2, 0},
                         Vec3D{l / 2, w / 2, h},
                         Vec3D{-l / 2, w / 2, h},
                         Vec3D{-l / 2, -w / 2, h},
                         Vec3D{l / 2, -w / 2, h}};
    box_mesh.indices = {0, 3, 1, 3, 2, 1, 4, 5, 7, 7, 5, 6, 7, 6, 3, 3, 6, 2, 5, 4, 1, 1, 4, 0, 0, 4, 7, 7, 3, 0, 1, 6, 5, 1, 2, 6};

    return box_mesh;
}

Mesh3D RoadObject::get_mesh(double eps) const
{
    auto road_ptr = this->road.lock();
    if (!road_ptr)
        throw std::runtime_error("could not access parent road for road object");

    std::vector<RoadObjectRepeat> repeats_copy = this->repeats;
    if (repeats_copy.empty())
        repeats_copy.push_back({NAN, 0, 1, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN});

    const Mat3D rot_mat = EulerAnglesToMatrix<double>(roll, pitch, hdg);

    auto get_t_s = [&](const RoadObjectRepeat& r, const double& p) -> double
    { return (isnan(r.t_start) || isnan(r.t_end)) ? this->t0 : r.t_start + p * (r.t_end - r.t_start); };

    auto get_z_s = [&](const RoadObjectRepeat& r, const double& p) -> double
    { return (isnan(r.z_offset_start) || isnan(r.z_offset_end)) ? this->z0 : r.z_offset_start + p * (r.z_offset_end - r.z_offset_start); };

    auto get_height_s = [&](const RoadObjectRepeat& r, const double& p) -> double
    { return (isnan(r.height_start) || isnan(r.height_end)) ? this->height : r.height_start + p * (r.height_end - r.height_start); };

    auto get_width_s = [&](const RoadObjectRepeat& r, const double& p) -> double
    { return (isnan(r.width_start) || isnan(r.width_end)) ? this->width : r.width_start + p * (r.width_end - r.width_start); };

    Mesh3D road_obj_mesh;

    for (const RoadObjectRepeat& repeat : repeats_copy)
    {
        const double s_start = isnan(repeat.s0) ? this->s0 : repeat.s0;
        const double s_end = std::min(s_start + repeat.length, road_ptr->length);

        if (repeat.distance != 0)
        {
            for (double s = s_start; s <= s_end; s += repeat.distance)
            {
                const double p = (s_end == s_start) ? 1.0 : (s - s_start) / (s_end - s_start);
                const double t_s = get_t_s(repeat, p);
                const double z_s = get_z_s(repeat, p);
                const double height_s = get_height_s(repeat, p);
                const double width_s = get_width_s(repeat, p);

                Mesh3D single_road_obj_mesh;
                if (this->radius > 0)
                {
                    single_road_obj_mesh = this->get_cylinder(eps, this->radius, height_s);
                }
                else if (width_s > 0 && this->length > 0)
                {
                    single_road_obj_mesh = this->get_box(width_s, this->length, height_s);
                }

                Vec3D       e_s, e_t, e_h;
                const Vec3D p0 = road_ptr->get_xyz(s, t_s, z_s, &e_s, &e_t, &e_h);
                const Mat3D base_mat{{{e_s[0], e_t[0], e_h[0]}, {e_s[1], e_t[1], e_h[1]}, {e_s[2], e_t[2], e_h[2]}}};
                for (Vec3D& pt_uvz : single_road_obj_mesh.vertices)
                {
                    pt_uvz = MatVecMultiplication(rot_mat, pt_uvz);
                    pt_uvz = MatVecMultiplication(base_mat, pt_uvz);
                    pt_uvz = add(pt_uvz, p0);
                }

                road_obj_mesh.add_mesh(single_road_obj_mesh);
            }
        }
        else
        {
            Mesh3D continuous_road_obj_mesh;

            const std::array<size_t, 24> idx_patch_template = {1, 5, 4, 1, 4, 0, 2, 7, 6, 2, 3, 7, 1, 6, 5, 1, 2, 6, 0, 4, 7, 0, 7, 3};
            for (const double& s : road_ptr->ref_line->approximate_linear(eps, s_start, s_end))
            {
                const double p = (s_end == s_start) ? 1.0 : (s - s_start) / (s_end - s_start);
                const double t_s = get_t_s(repeat, p);
                const double z_s = get_z_s(repeat, p);
                const double height_s = get_height_s(repeat, p);
                const double width_s = get_width_s(repeat, p);

                continuous_road_obj_mesh.vertices.push_back(road_ptr->get_xyz(s, t_s - 0.5 * width_s, z_s));
                continuous_road_obj_mesh.vertices.push_back(road_ptr->get_xyz(s, t_s + 0.5 * width_s, z_s));
                continuous_road_obj_mesh.vertices.push_back(road_ptr->get_xyz(s, t_s + 0.5 * width_s, z_s + height_s));
                continuous_road_obj_mesh.vertices.push_back(road_ptr->get_xyz(s, t_s - 0.5 * width_s, z_s + height_s));

                if (continuous_road_obj_mesh.vertices.size() == 4)
                {
                    const std::array<size_t, 6> front_idx_patch = {0, 2, 1, 0, 3, 2};
                    continuous_road_obj_mesh.indices.insert(continuous_road_obj_mesh.indices.end(), front_idx_patch.begin(), front_idx_patch.end());
                }

                if (continuous_road_obj_mesh.vertices.size() > 7)
                {
                    const size_t           cur_offs = continuous_road_obj_mesh.vertices.size() - 8;
                    std::array<size_t, 24> wall_idx_patch;
                    for (size_t idx = 0; idx < idx_patch_template.size(); idx++)
                        wall_idx_patch.at(idx) = idx_patch_template.at(idx) + cur_offs;
                    continuous_road_obj_mesh.indices.insert(continuous_road_obj_mesh.indices.end(), wall_idx_patch.begin(), wall_idx_patch.end());
                }
            }

            const size_t                last_idx = continuous_road_obj_mesh.vertices.size() - 1;
            const std::array<size_t, 6> back_idx_patch = {last_idx - 3, last_idx - 2, last_idx - 1, last_idx - 3, last_idx - 1, last_idx};
            continuous_road_obj_mesh.indices.insert(continuous_road_obj_mesh.indices.end(), back_idx_patch.begin(), back_idx_patch.end());

            road_obj_mesh.add_mesh(continuous_road_obj_mesh);
        }
    }

    if (this->local_outline.size() > 1)
    {
        Mesh3D outline_road_obj_mesh;
        for (const RoadObjectCornerLocal& corner_local : this->local_outline)
        {
            outline_road_obj_mesh.vertices.push_back({corner_local.u, corner_local.v, corner_local.z});
            outline_road_obj_mesh.vertices.push_back({corner_local.u, corner_local.v, corner_local.z + corner_local.height});
            if (outline_road_obj_mesh.vertices.size() > 3)
            {
                const size_t                cur_idx = outline_road_obj_mesh.vertices.size() - 1;
                const std::array<size_t, 6> wall_idx_patch = {cur_idx, cur_idx - 3, cur_idx - 1, cur_idx, cur_idx - 2, cur_idx - 3};
                outline_road_obj_mesh.indices.insert(outline_road_obj_mesh.indices.end(), wall_idx_patch.begin(), wall_idx_patch.end());
            }
        }

        Vec3D e_s, e_t, e_h;
        Vec3D p0 = road_ptr->get_xyz(this->s0, this->t0, this->z0, &e_s, &e_t, &e_h);
        p0[2] = 0.0;

        const Mat3D base_mat{{{e_s[0], e_t[0], e_h[0]}, {e_s[1], e_t[1], e_h[1]}, {e_s[2], e_t[2], e_h[2]}}};
        for (Vec3D& pt_uvz : outline_road_obj_mesh.vertices)
        {
            pt_uvz = MatVecMultiplication(rot_mat, pt_uvz);
            pt_uvz = MatVecMultiplication(base_mat, pt_uvz);
            pt_uvz = add(pt_uvz, p0);
        }

        const size_t                last_idx = outline_road_obj_mesh.vertices.size() - 1;
        const std::array<size_t, 6> last_idx_patch = {0, last_idx - 1, last_idx, 0, last_idx, 1};
        outline_road_obj_mesh.indices.insert(outline_road_obj_mesh.indices.end(), last_idx_patch.begin(), last_idx_patch.end());

        road_obj_mesh.add_mesh(outline_road_obj_mesh);
    }

    return road_obj_mesh;
}
} // namespace odr