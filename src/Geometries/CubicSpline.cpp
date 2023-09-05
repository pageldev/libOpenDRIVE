#include "Geometries/CubicSpline.h"
#include "CubicBezier.hpp"
#include "Math.hpp"
#include "Utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace odr
{
Poly3::Poly3(double s0, double a, double b, double c, double d)
{
    /* ds = s - s0 => resolve to polynomial form */
    /* make poly3s work on absolute s position => makes CubicSpline::add work */
    this->a = a - b * s0 + c * s0 * s0 - d * s0 * s0 * s0;
    this->b = b - 2 * c * s0 + 3 * d * s0 * s0;
    this->c = c - 3 * d * s0;
    this->d = d;
}

double Poly3::get(double s) const { return a + b * s + c * s * s + d * s * s * s; }

double Poly3::get_grad(double s) const { return b + 2 * c * s + 3 * d * s * s; }

double Poly3::get_max(double s_start, double s_end) const
{
    if (this->d != 0)
    {
        const double s_extr = (std::sqrt(c * c - 3 * b * d) - c) / (3 * d);
        const double max_val1 = this->get(std::min(std::max(s_extr, s_start), s_end));
        const double max_val2 = this->get(std::min(std::max(-s_extr, s_start), s_end));
        return std::max(max_val1, max_val2);
    }
    else if (this->c != 0)
    {
        const double s_extr = (-b) / (2 * c);
        return this->get(std::min(std::max(s_extr, s_start), s_end));
    }

    return this->get(s_start);
}

std::set<double> Poly3::approximate_linear(double eps, double s_start, double s_end) const
{
    if (s_start == s_end)
        return {};

    if (d == 0 && c == 0)
        return {s_start, s_end};

    std::vector<double> s_vals;
    if (d == 0 && c != 0)
    {
        double s = s_start;
        while (s < s_end)
        {
            s_vals.push_back(s);
            s = (c != 0) ? s + std::sqrt(std::abs(eps / c)) : s + eps;
        }
    }
    else
    {
        /* transform to parametric form */
        const double& s_0 = s_start;
        const double& s_1 = s_end;
        const double  d_p = -d * s_0 * s_0 * s_0 + d * s_1 * s_1 * s_1 - 3 * d * s_0 * s_1 * s_1 + 3 * d * s_0 * s_0 * s_1;
        const double  c_p =
            3 * d * s_0 * s_0 * s_0 + 3 * d * s_0 * s_1 * s_1 - 6 * d * s_0 * s_0 * s_1 + c * s_0 * s_0 + c * s_1 * s_1 - 2 * c * s_0 * s_1;
        const double b_p = -3 * d * s_0 * s_0 * s_0 + 3 * d * s_0 * s_0 * s_1 - 2 * c * s_0 * s_0 + 2 * c * s_0 * s_1 - b * s_0 + b * s_1;
        const double a_p = d * s_0 * s_0 * s_0 + c * s_0 * s_0 + b * s_0 + a;

        const std::array<Vec1D, 4> coefficients = {{{a_p}, {b_p}, {c_p}, {d_p}}};
        const std::set<double>     p_vals = CubicBezier1D(CubicBezier1D::get_control_points(coefficients)).approximate_linear(eps);

        s_vals.push_back(s_start);
        for (const double& p : p_vals)
            s_vals.push_back(p * (s_end - s_start) + s_start);
    }

    if ((s_end - s_vals.back()) < 1e-9 && (s_vals.size() != 1))
        s_vals.back() = s_end;
    else
        s_vals.push_back(s_end);

    std::set<double> s_vals_set(s_vals.begin(), s_vals.end());
    return s_vals_set;
}

void Poly3::negate()
{
    a = -a;
    b = -b;
    c = -c;
    d = -d;
}

bool Poly3::isnan() const { return (std::isnan(this->a) || std::isnan(this->b) || std::isnan(this->c) || std::isnan(this->d)); }

bool CubicSpline::empty() const { return this->s0_to_poly.empty(); }

std::size_t CubicSpline::size() const { return this->s0_to_poly.size(); }

double CubicSpline::get(double s, double default_val, bool extend_start) const
{
    const Poly3& poly = this->get_poly(s, extend_start);
    if (poly.isnan())
        return default_val;
    return poly.get(s);
}

double CubicSpline::get_grad(double s, double default_val, bool extend_start) const
{
    const Poly3& poly = this->get_poly(s, extend_start);
    if (poly.isnan())
        return default_val;
    return poly.get_grad(s);
}

CubicSpline CubicSpline::negate() const
{
    CubicSpline negated = *this;
    for (auto& s0_poly : negated.s0_to_poly)
        s0_poly.second.negate();
    return negated;
}

CubicSpline CubicSpline::add(const CubicSpline& other) const
{
    if (other.s0_to_poly.empty())
        return *this;
    if (this->s0_to_poly.empty())
        return other;

    std::set<double> s0_vals = get_map_keys(this->s0_to_poly);
    std::set<double> other_s0s = get_map_keys(other.s0_to_poly);
    s0_vals.insert(other_s0s.begin(), other_s0s.end());

    CubicSpline retval;
    for (const double& s0 : s0_vals)
    {
        const Poly3& this_poly = this->get_poly(s0, false);
        const Poly3& other_poly = other.get_poly(s0, false);

        if (this_poly.isnan() || other_poly.isnan()) // can't be both NAN
        {
            retval.s0_to_poly[s0] = this_poly.isnan() ? other_poly : this_poly;
            continue;
        }

        Poly3 res;
        res.a = this_poly.a + other_poly.a;
        res.b = this_poly.b + other_poly.b;
        res.c = this_poly.c + other_poly.c;
        res.d = this_poly.d + other_poly.d;
        retval.s0_to_poly[s0] = res;
    }
    return retval;
}

Poly3 CubicSpline::get_poly(double s, bool extend_start) const
{
    if (this->s0_to_poly.empty())
        return Poly3(NAN, NAN, NAN, NAN, NAN);

    if ((extend_start == false) && (s < this->s0_to_poly.begin()->first))
        return Poly3(NAN, NAN, NAN, NAN, NAN);

    // will return first poly if s < s_start and last poly for s > s_end
    auto target_poly_iter = this->s0_to_poly.upper_bound(s);
    if (target_poly_iter != this->s0_to_poly.begin())
        target_poly_iter--;
    return target_poly_iter->second;
}

double CubicSpline::get_max(double s_start, double s_end) const
{
    if ((s_start == s_end) || this->s0_to_poly.empty())
        return 0;

    auto s_end_poly_iter = this->s0_to_poly.lower_bound(s_end);
    auto s_start_poly_iter = this->s0_to_poly.upper_bound(s_start);
    if (s_start_poly_iter != this->s0_to_poly.begin())
        s_start_poly_iter--;

    std::vector<double> max_poly_vals;
    for (auto s_poly_iter = s_start_poly_iter; s_poly_iter != s_end_poly_iter; s_poly_iter++)
    {
        const double s_start_poly = std::max(s_poly_iter->first, s_start);
        const double s_end_poly = (std::next(s_poly_iter) == s_end_poly_iter) ? s_end : std::min(std::next(s_poly_iter)->first, s_end);
        max_poly_vals.push_back(s_poly_iter->second.get_max(s_start_poly, s_end_poly));
    }

    const auto   max_iter = std::max_element(max_poly_vals.begin(), max_poly_vals.end());
    const double max_val = (max_iter == max_poly_vals.end()) ? 0 : *max_iter;
    return max_val;
}

std::set<double> CubicSpline::approximate_linear(double eps, double s_start, double s_end) const
{
    if ((s_start == s_end) || this->s0_to_poly.empty())
        return {};

    auto s_end_poly_iter = this->s0_to_poly.lower_bound(s_end);
    auto s_start_poly_iter = this->s0_to_poly.upper_bound(s_start);
    if (s_start_poly_iter != this->s0_to_poly.begin())
        s_start_poly_iter--;

    std::set<double> s_vals;
    for (auto s_poly_iter = s_start_poly_iter; s_poly_iter != s_end_poly_iter; s_poly_iter++)
    {
        const double s_start_poly = std::max(s_poly_iter->first, s_start);
        const double s_end_poly = (std::next(s_poly_iter) == s_end_poly_iter) ? s_end : std::min(std::next(s_poly_iter)->first, s_end);

        std::set<double> s_vals_poly = s_poly_iter->second.approximate_linear(eps, s_start_poly, s_end_poly);
        if (s_vals_poly.size() < 2)
        {
            std::string err_msg = std::string("expected at least two sample points, got ") + std::to_string(s_vals_poly.size()) +
                                  std::string(" for [") + std::to_string(s_start_poly) + ' ' + std::to_string(s_end_poly) + ']';
            throw std::runtime_error(err_msg);
        }

        s_vals.insert(s_vals_poly.begin(), s_vals_poly.end());
    }

    return s_vals;
}

} // namespace odr
