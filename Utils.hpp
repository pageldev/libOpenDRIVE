#pragma once

#include "Math.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <set>
#include <type_traits>
#include <vector>

namespace odr
{
struct Mesh3D
{
    std::vector<Vec3D>  vertices;
    std::vector<size_t> indices;
};

template<class C, class T, T C::*member>
struct SharedPtrCmp
{
    bool operator()(const std::shared_ptr<C>& lhs, const std::shared_ptr<C>& rhs) const { return (*lhs).*member < (*rhs).*member; }
};

template<class K, class V>
std::set<K> extract_keys(std::map<K, V> const& input_map)
{
    std::set<K> retval;
    for (auto const& element : input_map)
        retval.insert(element.first);

    return retval;
}

inline Mesh3D generate_mesh_from_borders(const Line3D& inner_border, const Line3D& outer_border)
{
    Mesh3D out_mesh;

    if (inner_border.size() != outer_border.size())
        throw std::runtime_error("outer and inner border line should have equal number of points");

    out_mesh.vertices = outer_border;
    out_mesh.vertices.insert(out_mesh.vertices.end(), inner_border.rbegin(), inner_border.rend());

    const size_t num_pts = out_mesh.vertices.size();
    for (size_t l_idx = 1, r_idx = num_pts - 2; l_idx < (num_pts >> 1); l_idx++, r_idx--)
    {
        std::vector<size_t> indicies_patch = {l_idx, l_idx - 1, r_idx + 1, r_idx, l_idx, r_idx + 1};
        out_mesh.indices.insert(out_mesh.indices.end(), indicies_patch.begin(), indicies_patch.end());
    }

    return out_mesh;
}

template<typename T, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
T golden_section_search(const std::function<T(T)>& f, T a, T b, const T tol)
{
    const T invphi = (std::sqrt(5) - 1) / 2;
    const T invphi2 = (3 - std::sqrt(5)) / 2;

    T h = b - a;
    if (h <= tol)
        return 0.5 * (a + b);

    // Required steps to achieve tolerance
    int n = static_cast<int>(std::ceil(std::log(tol / h) / std::log(invphi)));

    T c = a + invphi2 * h;
    T d = a + invphi * h;
    T yc = f(c);
    T yd = f(d);

    for (int k = 0; k < (n - 1); k++)
    {
        if (yc < yd)
        {
            b = d;
            d = c;
            yd = yc;
            h = invphi * h;
            c = a + invphi2 * h;
            yc = f(c);
        }
        else
        {
            a = c;
            c = d;
            yc = yd;
            h = invphi * h;
            d = a + invphi * h;
            yd = f(d);
        }
    }

    if (yc < yd)
        return 0.5 * (a + d);

    return 0.5 * (c + b);
}

template<typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
void rdp(
    const std::vector<Vec<T, Dim>>& points, const T epsilon, std::vector<Vec<T, Dim>>& out, size_t start_idx = 0, size_t step = 1, int _end_idx = -1)
{
    size_t end_idx = (_end_idx > 0) ? static_cast<size_t>(_end_idx) : points.size();
    size_t last_idx = static_cast<size_t>((end_idx - start_idx - 1) / step) * step + start_idx;

    if ((last_idx + 1 - start_idx) < 2)
        return;

    /* find the point with the maximum distance from line BETWEEN start and end */
    T      d_max(0);
    size_t d_max_idx = 0;
    for (size_t idx = start_idx + step; idx < last_idx; idx += step)
    {
        std::array<T, Dim> delta;
        for (size_t dim = 0; dim < Dim; dim++)
            delta[dim] = points.at(last_idx)[dim] - points.at(start_idx)[dim];

        // Normalise
        T mag(0);
        for (size_t dim = 0; dim < Dim; dim++)
            mag += std::pow(delta.at(dim), 2.0);
        mag = std::sqrt(mag);
        if (mag > 0.0)
        {
            for (size_t dim = 0; dim < Dim; dim++)
                delta.at(dim) = delta.at(dim) / mag;
        }

        std::array<T, Dim> pv;
        for (size_t dim = 0; dim < Dim; dim++)
            pv[dim] = points.at(idx)[dim] - points.at(start_idx)[dim];

        // Get dot product (project pv onto normalized direction)
        T pvdot(0);
        for (size_t dim = 0; dim < Dim; dim++)
            pvdot += delta.at(dim) * pv.at(dim);

        // Scale line direction vector
        std::array<T, Dim> ds;
        for (size_t dim = 0; dim < Dim; dim++)
            ds[dim] = pvdot * delta.at(dim);

        // Subtract this from pv
        std::array<T, Dim> a;
        for (size_t dim = 0; dim < Dim; dim++)
            a[dim] = pv.at(dim) - ds.at(dim);

        T d(0);
        for (size_t dim = 0; dim < Dim; dim++)
            d += std::pow(a.at(dim), 2.0);
        d = std::sqrt(d);

        if (d > d_max)
        {
            d_max = d;
            d_max_idx = idx;
        }
    }

    if (d_max > epsilon)
    {
        std::vector<Vec<T, Dim>> rec_results_1;
        rdp<T, Dim>(points, epsilon, rec_results_1, start_idx, step, d_max_idx + 1);
        std::vector<Vec<T, Dim>> rec_results_2;
        rdp<T, Dim>(points, epsilon, rec_results_2, d_max_idx, step, end_idx);

        out.assign(rec_results_1.begin(), rec_results_1.end() - 1);
        out.insert(out.end(), rec_results_2.begin(), rec_results_2.end());
    }
    else
    {
        out.clear();
        out.push_back(points.at(start_idx));
        out.push_back(points.at(last_idx));
    }
}

template<typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
std::array<Vec<T, Dim>, 4> subdivide_cubic_bezier(T p_start, T p_end, const std::array<Vec<T, Dim>, 4>& ctrl_pts)
{
    /* modified f_cubic allowing different p values for segments */
    auto f_cubic_p123 = [&](const T& p1, const T& p2, const T& p3) -> Vec<T, Dim> {
        Vec<T, Dim> out;
        for (size_t dim = 0; dim < Dim; dim++)
        {
            out[dim] =
                (1 - p3) * ((1 - p2) * ((1 - p1) * ctrl_pts[0][dim] + p1 * ctrl_pts[1][dim]) + p2 * ((1 - p1) * ctrl_pts[1][dim] + p1 * ctrl_pts[2][dim])) +
                p3 * ((1 - p2) * ((1 - p1) * ctrl_pts[1][dim] + p1 * ctrl_pts[2][dim]) + p2 * ((1 - p1) * ctrl_pts[2][dim] + p1 * ctrl_pts[3][dim]));
        }
        return out;
    };

    std::array<Vec<T, Dim>, 4> ctrl_pts_sub;
    ctrl_pts_sub[0] = f_cubic_p123(p_start, p_start, p_start);
    ctrl_pts_sub[1] = f_cubic_p123(p_start, p_start, p_end);
    ctrl_pts_sub[2] = f_cubic_p123(p_start, p_end, p_end);
    ctrl_pts_sub[3] = f_cubic_p123(p_end, p_end, p_end);

    return ctrl_pts_sub;
}

template<typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
std::vector<T> approximate_linear_quad_bezier(const std::array<Vec<T, Dim>, 3>& ctrl_pts, T eps)
{
    Vec<T, Dim> param_c;
    for (size_t dim = 0; dim < Dim; dim++)
        param_c[dim] = ctrl_pts[0][dim] - 2 * ctrl_pts[1][dim] + ctrl_pts[2][dim];

    const T step_size = std::min(std::sqrt((4 * eps) / norm(param_c)), 1.0);

    std::vector<T> p_vals;
    for (T p = 0; p < 1; p += step_size)
        p_vals.push_back(p);
    if (p_vals.back() != 1)
        p_vals.push_back(1);

    return p_vals;
}

} // namespace odr