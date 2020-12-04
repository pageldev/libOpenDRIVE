#include "CubicSpline.h"

#include <cmath>
#include <utility>
#include <vector>

namespace odr
{
Poly3::Poly3(double s0, double a, double b, double c, double d) : s0(s0), a(a), b(b), c(c), d(d) {}

double Poly3::get(double s) const
{
    const double ds = s - s0;
    return a + b * ds + c * ds * ds + d * ds * ds * ds;
}

double Poly3::get_grad(double s) const { return b + 2 * c * s + 3 * d * s * s; }

double Poly3::get_max(std::pair<double, double> range) const
{
    std::vector<double> s_extremas;
    if (d != 0)
    {
        double s1_extr = -(std::sqrt(std::abs(c * c - 3 * b * d)) + c - 3 * d * s0) / (3 * d);
        s_extremas.push_back(std::min(std::max(s1_extr, range.first), range.second));
        double s2_extr = (std::sqrt(std::abs(c * c - 3 * b * d)) - c + 3 * d * s0) / (3 * d);
        s_extremas.push_back(std::min(std::max(s2_extr, range.first), range.second));
    }
    if (c != 0)
    {
        double s3_extr = s0 - (b / (2 * c));
        s_extremas.push_back(std::min(std::max(s3_extr, range.first), range.second));
    }

    std::vector<double> max_vals;
    for (const double& s_extrema : s_extremas)
        max_vals.push_back(this->get(s_extrema));

    const auto   max_iter = std::max_element(max_vals.begin(), max_vals.end());
    const double max_val = (max_iter == max_vals.end()) ? 0 : *max_iter;
    return max_val;
}

size_t CubicSpline::size() const { return this->s0_to_poly.size(); }

double CubicSpline::get(double s) const
{
    std::shared_ptr<const Poly3> poly = this->get_poly(s);
    if (poly)
        return poly->get(s);
    return 0;
}

double CubicSpline::get_grad(double s) const
{
    std::shared_ptr<const Poly3> poly = this->get_poly(s);
    if (poly)
        return poly->get_grad(s);
    return 0;
}

double CubicSpline::get_max(std::pair<double, double> range) const
{
    std::vector<double> max_vals;
    for (auto s0_poly_iter = this->s0_to_poly.begin(); s0_poly_iter != this->s0_to_poly.end(); s0_poly_iter++)
    {
        const auto   next_s0_poly = std::next(s0_poly_iter);
        const double upper_bound = (next_s0_poly == this->s0_to_poly.end()) ? range.second : next_s0_poly->first;
        const double lower_bound = (s0_poly_iter == this->s0_to_poly.begin()) ? range.first : s0_poly_iter->first;

        max_vals.push_back(s0_poly_iter->second->get_max({lower_bound, upper_bound}));
    }

    const auto   max_iter = std::max_element(max_vals.begin(), max_vals.end());
    const double max_val = (max_iter == max_vals.end()) ? 0 : *max_iter;
    return max_val;
}

std::shared_ptr<const Poly3> CubicSpline::get_poly(double s) const
{
    if (this->s0_to_poly.size() > 0)
    {
        auto target_poly_iter = this->s0_to_poly.upper_bound(s);
        if (target_poly_iter != this->s0_to_poly.begin())
            target_poly_iter--;
        return target_poly_iter->second;
    }
    return nullptr;
}

} // namespace odr