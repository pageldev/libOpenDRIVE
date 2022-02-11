#include "Math.hpp"
#include "Utils.hpp"

#include <array>
#include <cmath>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <stdio.h>

namespace odr
{
template<typename T, size_t Dim>
struct CubicBezier
{
    CubicBezier() = default;
    CubicBezier(std::array<Vec<T, Dim>, 4> control_points);

    Vec<T, Dim>                get(T t) const;
    Vec<T, Dim>                get_grad(T t) const;
    T                          get_t(T arclen) const;
    T                          get_length() const;
    std::array<Vec<T, Dim>, 4> get_subcurve(T t_start, T t_end) const;
    std::set<T>                approximate_linear(T eps) const;

    static std::array<Vec<T, Dim>, 4> get_control_points(const std::array<Vec<T, Dim>, 4> coefficients)
    {
        /* a + b*x + c*x^2 +d*x^3 */
        const Vec<T, Dim>& a = coefficients[0];
        const Vec<T, Dim>& b = coefficients[1];
        const Vec<T, Dim>& c = coefficients[2];
        const Vec<T, Dim>& d = coefficients[3];

        std::array<Vec<T, Dim>, 4> ctrl_pts;
        ctrl_pts[0] = a;

        for (size_t dim = 0; dim < Dim; dim++)
            ctrl_pts[1][dim] = (b[dim] / 3) + a[dim];

        for (size_t dim = 0; dim < Dim; dim++)
            ctrl_pts[2][dim] = (c[dim] / 3) + 2 * ctrl_pts[1][dim] - ctrl_pts[0][dim];

        for (size_t dim = 0; dim < Dim; dim++)
            ctrl_pts[3][dim] = d[dim] + 3 * ctrl_pts[2][dim] - 3 * ctrl_pts[1][dim] + ctrl_pts[0][dim];

        return ctrl_pts;
    }

    static std::array<Vec<T, Dim>, 4> get_coefficients(const std::array<Vec<T, Dim>, 4> control_points)
    {
        const Vec<T, Dim>& pA = control_points[0];
        const Vec<T, Dim>& pB = control_points[1];
        const Vec<T, Dim>& pC = control_points[2];
        const Vec<T, Dim>& pD = control_points[3];

        std::array<Vec<T, Dim>, 4> coefficients;
        coefficients[0] = pA;

        for (size_t dim = 0; dim < Dim; dim++)
            coefficients[1][dim] = 3 * pB[dim] - 3 * pA[dim];

        for (size_t dim = 0; dim < Dim; dim++)
            coefficients[2][dim] = 3 * pC[dim] - 6 * pB[dim] + 3 * pA[dim];

        for (size_t dim = 0; dim < Dim; dim++)
            coefficients[3][dim] = pD[dim] - 3 * pC[dim] + 3 * pB[dim] - pA[dim];

        return coefficients;
    }

    T valid_length;

    std::array<Vec<T, Dim>, 4> control_points;
    std::map<T, T>             arclen_t;
    static const double        LengthTolerance;
};

template<typename T, size_t Dim>
CubicBezier<T, Dim>::CubicBezier(std::array<Vec<T, Dim>, 4> control_points) : control_points(control_points)
{
    const std::set<T> t_vals = this->approximate_linear(this->LengthTolerance);
    if (t_vals.size() < 2)
        throw std::runtime_error("expected at least two t values");

    arclen_t[T(0)] = T(0);

    T arclen(0);
    for (auto t_val_iter = std::next(t_vals.begin()); t_val_iter != t_vals.end(); t_val_iter++)
    {
        const Vec<T, Dim> pt_prev = this->get(*std::prev(t_val_iter));
        const Vec<T, Dim> pt = this->get(*t_val_iter);
        arclen += euclDistance(pt, pt_prev);
        this->arclen_t[arclen] = *t_val_iter;
    }

    this->valid_length = std::prev(this->arclen_t.end())->first;
}

template<typename T, size_t Dim>
Vec<T, Dim> CubicBezier<T, Dim>::get(T t) const
{
    Vec<T, Dim> out_pt;
    for (size_t dim = 0; dim < Dim; dim++)
        out_pt[dim] = (1 - t) * (1 - t) * (1 - t) * control_points[0][dim] + 3 * t * (1 - t) * (1 - t) * control_points[1][dim] +
                      3 * t * t * (1 - t) * control_points[2][dim] + t * t * t * control_points[3][dim];
    return out_pt;
}

template<typename T, size_t Dim>
T CubicBezier<T, Dim>::get_t(T arclen) const
{
    if ((arclen - this->valid_length) > this->LengthTolerance || arclen < 0)
    {
        std::stringstream ss_err;
        ss_err << "arclength " << arclen << " out of range; valid length: " << this->valid_length;
        throw std::runtime_error(ss_err.str());
    }

    arclen = std::min<T>(arclen, this->valid_length);

    auto arclen_t_iter = this->arclen_t.upper_bound(arclen);
    if (arclen_t_iter != this->arclen_t.begin())
        arclen_t_iter--;

    const T arcl_lower_bound = arclen_t_iter->first;
    const T t_lower_bound = arclen_t_iter->second;
    if (arclen == arcl_lower_bound)
        return t_lower_bound;

    const T arcl_upper_bound = std::next(arclen_t_iter)->first;
    const T t_upper_bound = std::next(arclen_t_iter)->second;
    const T seg_arc_len = arcl_upper_bound - arcl_lower_bound;
    const T seg_t_len = t_upper_bound - t_lower_bound;

    return t_lower_bound + ((arclen - arcl_lower_bound) / seg_arc_len) * seg_t_len;
}

template<typename T, size_t Dim>
T CubicBezier<T, Dim>::get_length() const
{
    return std::prev(arclen_t.end())->first;
}

template<typename T, size_t Dim>
Vec<T, Dim> CubicBezier<T, Dim>::get_grad(T t) const
{
    std::array<Vec<T, Dim>, 4> coefficients = this->get_coefficients(this->control_points);

    Vec<T, Dim> grad;
    for (size_t dim = 0; dim < Dim; dim++)
        grad[dim] = coefficients[1][dim] + 2 * coefficients[2][dim] * t + 3 * coefficients[3][dim] * t * t;

    return grad;
}

template<typename T, size_t Dim>
std::array<Vec<T, Dim>, 4> CubicBezier<T, Dim>::get_subcurve(T t_start, T t_end) const
{
    /* modified get(T t) allowing different t values for segments */
    auto f_cubic_t123 = [](const T& t1, const T& t2, const T& t3, const std::array<Vec<T, Dim>, 4>& ctrl_pts) -> Vec<T, Dim>
    {
        Vec<T, Dim> out;
        for (size_t dim = 0; dim < Dim; dim++)
        {
            out[dim] =
                (1 - t3) *
                    ((1 - t2) * ((1 - t1) * ctrl_pts[0][dim] + t1 * ctrl_pts[1][dim]) + t2 * ((1 - t1) * ctrl_pts[1][dim] + t1 * ctrl_pts[2][dim])) +
                t3 * ((1 - t2) * ((1 - t1) * ctrl_pts[1][dim] + t1 * ctrl_pts[2][dim]) + t2 * ((1 - t1) * ctrl_pts[2][dim] + t1 * ctrl_pts[3][dim]));
        }
        return out;
    };

    std::array<Vec<T, Dim>, 4> ctrl_pts_sub;
    ctrl_pts_sub[0] = f_cubic_t123(t_start, t_start, t_start, control_points);
    ctrl_pts_sub[1] = f_cubic_t123(t_start, t_start, t_end, control_points);
    ctrl_pts_sub[2] = f_cubic_t123(t_start, t_end, t_end, control_points);
    ctrl_pts_sub[3] = f_cubic_t123(t_end, t_end, t_end, control_points);

    return ctrl_pts_sub;
}

template<typename T, size_t Dim>
std::set<T> CubicBezier<T, Dim>::approximate_linear(T eps) const
{
    /* approximate cubic bezier by splitting into quadratic ones */
    std::array<Vec<T, Dim>, 4> coefficients = this->get_coefficients(this->control_points);
    const T                    seg_size = std::pow(0.5 * eps / ((1.0 / 54.0) * norm(coefficients[3])), (1.0 / 3.0));

    std::vector<std::array<T, 2>> seg_intervals;
    for (T t = 0; t < 1; t += seg_size)
        seg_intervals.push_back({t, std::min<T>(t + seg_size, 1)});

    if (T(1) - (seg_intervals.back().at(1)) < 1e-6)
        seg_intervals.back().at(1) = T(1);
    else
        seg_intervals.push_back({seg_intervals.back().at(1), T(1)});

    std::vector<T> t_vals{0};
    for (const std::array<T, 2>& seg_intrvl : seg_intervals)
    {
        /* get sub-cubic bezier for interval */
        const double& t0 = seg_intrvl.at(0);
        const double& t1 = seg_intrvl.at(1);

        const std::array<Vec<T, Dim>, 4> c_pts_sub = this->get_subcurve(t0, t1);

        /* approximate sub-cubic bezier by two quadratic ones */
        Vec<T, Dim> pB_quad_0;
        for (size_t dim = 0; dim < Dim; dim++)
            pB_quad_0[dim] = (1.0 - 0.75) * c_pts_sub[0][dim] + 0.75 * c_pts_sub[1][dim];
        Vec<T, Dim> pB_quad_1;
        for (size_t dim = 0; dim < Dim; dim++)
            pB_quad_1[dim] = (1.0 - 0.75) * c_pts_sub[3][dim] + 0.75 * c_pts_sub[2][dim];
        Vec<T, Dim> pM_quad;
        for (size_t dim = 0; dim < Dim; dim++)
            pM_quad[dim] = (1.0 - 0.5) * pB_quad_0[dim] + 0.5 * pB_quad_1[dim];

        /* linear approximate the two quadratic bezier */
        for (const double& p_sub : approximate_linear_quad_bezier<T, Dim>({c_pts_sub[0], pB_quad_0, pM_quad}, 0.5 * eps))
            t_vals.push_back(t0 + p_sub * (t1 - t0) * 0.5);
        t_vals.pop_back();
        for (const double& p_sub : approximate_linear_quad_bezier<T, Dim>({pM_quad, pB_quad_1, c_pts_sub[3]}, 0.5 * eps))
            t_vals.push_back(t0 + (t1 - t0) * 0.5 + p_sub * (t1 - t0) * 0.5);
        t_vals.pop_back();
    }
    t_vals.push_back(1);

    return std::set<T>(t_vals.begin(), t_vals.end());
}

template<typename T, size_t Dim>
const double CubicBezier<T, Dim>::LengthTolerance = 1e-2;

typedef CubicBezier<double, 2> CubicBezier2D;
typedef CubicBezier<double, 1> CubicBezier1D;

} // namespace odr