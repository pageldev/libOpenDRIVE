#include "Lanes.h"
#include "RefLine.h"
#include "Road.h"
#include "Utils.hpp"

namespace odr
{
Lane::Lane(int id, bool level, std::string type) : id(id), level(level), type(type) {}

Vec3D Lane::get_surface_pt(double s, double t) const
{
    if (auto road_ptr = this->road.lock())
    {
        const double t_inner_brdr = this->inner_border.get(s);
        double       h_t = 0;

        if (this->level)
        {
            const double h_inner_brdr = -std::tan(road_ptr->crossfall.get_crossfall(s, (this->id > 0))) * std::abs(t_inner_brdr);
            const double superelev = road_ptr->superelevation.get(s); // cancel out superelevation
            h_t = h_inner_brdr + std::tan(superelev) * (t - t_inner_brdr);
        }
        else
        {
            h_t = -std::tan(road_ptr->crossfall.get_crossfall(s, (this->id > 0))) * std::abs(t);
        }

        if (this->s_to_height_offset.size() > 0)
        {
            const std::map<double, HeightOffset>& height_offs = this->s_to_height_offset;

            auto s0_height_offs_iter = height_offs.upper_bound(s);
            if (s0_height_offs_iter != height_offs.begin())
                s0_height_offs_iter--;

            const double t_outer_brdr = this->outer_border.get(s);
            const double inner_height = s0_height_offs_iter->second.inner;
            const double outer_height = s0_height_offs_iter->second.outer;
            const double p_t = (t_outer_brdr != t_inner_brdr) ? (t - t_inner_brdr) / (t_outer_brdr - t_inner_brdr) : 0.0;
            h_t += p_t * (outer_height - inner_height) + inner_height;

            if (std::next(s0_height_offs_iter) != height_offs.end())
            {
                /* if successive lane height entry available linearly interpolate */
                const double ds = std::next(s0_height_offs_iter)->first - s0_height_offs_iter->first;
                const double d_lh_inner = std::next(s0_height_offs_iter)->second.inner - inner_height;
                const double dh_inner = (d_lh_inner / ds) * (s - s0_height_offs_iter->first);
                const double d_lh_outer = std::next(s0_height_offs_iter)->second.outer - outer_height;
                const double dh_outer = (d_lh_outer / ds) * (s - s0_height_offs_iter->first);

                h_t += p_t * (dh_outer - dh_inner) + dh_inner;
            }
        }

        return road_ptr->get_xyz(s, t, h_t);
    }
    else
    {
        throw std::runtime_error("could not access parent road for lane section");
    }

    return {};
}

Line3D Lane::get_border_line(double s_start, double s_end, double eps, bool outer, bool fixed_sample_dist) const
{
    std::set<double> s_vals;
    if (auto road_ptr = this->road.lock())
    {
        if (fixed_sample_dist)
        {
            for (double s = s_start; s < s_end; s += eps)
                s_vals.insert(s);
            s_vals.insert(s_end);
        }
        else
        {
            s_vals = road_ptr->ref_line->approximate_linear(eps, s_start, s_end);

            const CubicSpline& border = outer ? this->outer_border : this->inner_border;
            std::set<double>   s_vals_brdr = border.approximate_linear(eps, s_start, s_end);
            s_vals.insert(s_vals_brdr.begin(), s_vals_brdr.end());

            std::vector<double> s_vals_lane_height = extract_keys(this->s_to_height_offset);
            s_vals.insert(s_vals_lane_height.begin(), s_vals_lane_height.end());

            const double     t_max = this->outer_border.get_max(s_start, s_end);
            std::set<double> s_vals_superelev = road_ptr->superelevation.approximate_linear(std::atan(eps / std::abs(t_max)), s_start, s_end);
            s_vals.insert(s_vals_superelev.begin(), s_vals_superelev.end());
        }
    }
    else
    {
        throw std::runtime_error("could not access parent road for lane");
    }

    Line3D border_line;
    for (const double& s : s_vals)
    {
        const double t = outer ? this->outer_border.get(s) : this->inner_border.get(s);
        border_line.push_back(this->get_surface_pt(s, t));
    }

    return border_line;
}

Mesh3D Lane::get_mesh(double s_start, double s_end, double eps, bool fixed_sample_dist) const
{
    std::set<double> s_vals;
    if (auto road_ptr = this->road.lock())
    {
        if (fixed_sample_dist)
        {
            for (double s = s_start; s < s_end; s += eps)
                s_vals.insert(s);
            s_vals.insert(s_end);
        }
        else
        {
            s_vals = road_ptr->ref_line->approximate_linear(eps, s_start, s_end);

            std::set<double> s_vals_outer_brdr = this->outer_border.approximate_linear(eps, s_start, s_end);
            s_vals.insert(s_vals_outer_brdr.begin(), s_vals_outer_brdr.end());
            std::set<double> s_vals_inner_brdr = this->inner_border.approximate_linear(eps, s_start, s_end);
            s_vals.insert(s_vals_inner_brdr.begin(), s_vals_inner_brdr.end());

            std::vector<double> s_vals_lane_height = extract_keys(this->s_to_height_offset);
            s_vals.insert(s_vals_lane_height.begin(), s_vals_lane_height.end());

            const double     t_max = this->outer_border.get_max(s_start, s_end);
            std::set<double> s_vals_superelev = road_ptr->superelevation.approximate_linear(std::atan(eps / std::abs(t_max)), s_start, s_end);
            s_vals.insert(s_vals_superelev.begin(), s_vals_superelev.end());

            /* thin out s_vals array, be removing s vals closer than eps to each other */
            for (auto s_iter = s_vals.begin(); s_iter != s_vals.end();)
            {
                if (std::next(s_iter) != s_vals.end() && std::next(s_iter, 2) != s_vals.end() && ((*std::next(s_iter)) - *s_iter) <= eps)
                    s_iter = std::prev(s_vals.erase(std::next(s_iter)));
                else
                    s_iter++;
            }
        }
    }
    else
    {
        throw std::runtime_error("could not access parent road for lane");
    }

    Line3D inner_border_line;
    Line3D outer_border_line;
    for (const double& s : s_vals)
    {
        const double t_inner_brdr = this->inner_border.get(s);
        const double t_outer_brdr = this->outer_border.get(s);
        inner_border_line.push_back(this->get_surface_pt(s, t_inner_brdr));
        outer_border_line.push_back(this->get_surface_pt(s, t_outer_brdr));
    }

    return generate_mesh_from_borders(inner_border_line, outer_border_line);
}

} // namespace odr