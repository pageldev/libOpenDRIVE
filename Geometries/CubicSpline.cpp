#include "CubicSpline.h"
#include "Utils.hpp"

#include <cmath>
#include <utility>
#include <vector>

namespace odr
{
Poly3::Poly3(double s_start, double a, double b, double c, double d) : s_start(s_start)
{
    this->a = a - b * s_start + c * s_start * s_start - d * s_start * s_start * s_start;
    this->b = b - 2 * c * s_start + 3 * d * s_start * s_start;
    this->c = c - 3 * d * s_start;
    this->d = d;
}

double Poly3::get(double s) const
{
    // const double ds = s - s_start;
    return a + b * s + c * s * s + d * s * s * s;
}

double Poly3::get_grad(double s) const { return b + 2 * c * s + 3 * d * s * s; }

double Poly3::get_max(std::pair<double, double> range) const
{
    std::vector<double> s_extremas;
    if (d != 0)
    {
        double s1_extr = -(std::sqrt(std::abs(c * c - 3 * b * d)) + c - 3 * d * s_start) / (3 * d);
        s_extremas.push_back(std::min(std::max(s1_extr, range.first), range.second));
        double s2_extr = (std::sqrt(std::abs(c * c - 3 * b * d)) - c + 3 * d * s_start) / (3 * d);
        s_extremas.push_back(std::min(std::max(s2_extr, range.first), range.second));
    }
    if (c != 0)
    {
        double s3_extr = s_start - (b / (2 * c));
        s_extremas.push_back(std::min(std::max(s3_extr, range.first), range.second));
    }

    std::vector<double> max_vals;
    for (const double& s_extrema : s_extremas)
        max_vals.push_back(this->get(s_extrema));

    const auto   max_iter = std::max_element(max_vals.begin(), max_vals.end());
    const double max_val = (max_iter == max_vals.end()) ? 0 : *max_iter;
    return max_val;
}

void Poly3::negate()
{
    a = -a;
    b = -b;
    c = -c;
    d = -d;
}

size_t CubicSpline::size() const { return this->s_start_to_poly.size(); }

double CubicSpline::get(double s) const
{
    const double ds = s - s0;
    const Poly3  poly = this->get_poly(ds);
    return poly.get(ds);
}

double CubicSpline::get_grad(double s) const
{
    const double ds = s - s0;
    const Poly3  poly = this->get_poly(ds);
    return poly.get_grad(ds);
}

double CubicSpline::get_max(std::pair<double, double> range) const
{
    std::vector<double> max_vals;
    for (auto s_start_poly_iter = this->s_start_to_poly.begin(); s_start_poly_iter != this->s_start_to_poly.end(); s_start_poly_iter++)
    {
        const auto   next_s_start_poly = std::next(s_start_poly_iter);
        const double upper_bound = (next_s_start_poly == this->s_start_to_poly.end()) ? range.second : next_s_start_poly->first;
        const double lower_bound = (s_start_poly_iter == this->s_start_to_poly.begin()) ? range.first : s_start_poly_iter->first;

        max_vals.push_back(s_start_poly_iter->second.get_max({lower_bound, upper_bound}));
    }

    const auto   max_iter = std::max_element(max_vals.begin(), max_vals.end());
    const double max_val = (max_iter == max_vals.end()) ? 0 : *max_iter;
    return max_val;
}

CubicSpline CubicSpline::negate() const
{
    CubicSpline negated = *this;
    for (auto& s0_poly : negated.s_start_to_poly)
        s0_poly.second.negate();
    return negated;
}

CubicSpline CubicSpline::add(const CubicSpline& other) const
{
    std::set<double> s_start_vals = extract_keys(this->s_start_to_poly);
    std::set<double> other_s_starts = extract_keys(other.s_start_to_poly);
    s_start_vals.insert(other_s_starts.begin(), other_s_starts.end());

    CubicSpline retval;
    for (const double& s_start : s_start_vals)
    {
        const Poly3 this_poly = this->get_poly(s_start);
        const Poly3 other_poly = other.get_poly(s_start);

        Poly3 res;
        res.a = this_poly.a + other_poly.a;
        res.b = this_poly.b + other_poly.b;
        res.c = this_poly.c + other_poly.c;
        res.d = this_poly.d + other_poly.d;
        res.s_start = s_start;
        retval.s_start_to_poly[s_start] = res;
    }
    return retval;
}

Poly3 CubicSpline::get_poly(double s) const
{
    if (this->s_start_to_poly.size() > 0)
    {
        auto target_poly_iter = this->s_start_to_poly.upper_bound(s);
        if (target_poly_iter != this->s_start_to_poly.begin())
            target_poly_iter--;
        return target_poly_iter->second;
    }
    return Poly3();
}

} // namespace odr