#include "libodr/Geometries/CubicSpline.h"
#include "libodr/CubicBezier.hpp"
#include "libodr/Math.hpp"
#include "libodr/Utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace odr
{

CubicPoly::CubicPoly(double a, double b, double c, double d, double s_origin)
{
    require_or_throw(!std::isnan(a), "a must not be NaN");
    require_or_throw(!std::isnan(b), "b must not be NaN");
    require_or_throw(!std::isnan(c), "c must not be NaN");
    require_or_throw(!std::isnan(d), "d must not be NaN");
    require_or_throw(!std::isnan(s_origin), "s origin must not be NaN");

    // ds = s - s_origin => resolve to polynomial form
    // make CubicPolys work on absolute s position => makes CubicProfile::add work
    this->a = a - b * s_origin + c * s_origin * s_origin - d * s_origin * s_origin * s_origin;
    this->b = b - 2 * c * s_origin + 3 * d * s_origin * s_origin;
    this->c = c - 3 * d * s_origin;
    this->d = d;
}

double CubicPoly::evaluate(double s) const
{
    return a + b * s + c * s * s + d * s * s * s;
}

double CubicPoly::derivative(double s) const
{
    return b + 2 * c * s + 3 * d * s * s;
}

double CubicPoly::max_abs_value(double s_start, double s_end) const
{
    double max_abs_value = std::max(std::abs(this->evaluate(s_start)), std::abs(this->evaluate(s_end)));

    const auto update_max_abs_value = [&](double s)
    {
        if (s >= s_start && s <= s_end)
            max_abs_value = std::max(max_abs_value, std::abs(this->evaluate(s)));
    };

    if (this->d != 0)
    {
        const double discriminant = this->c * this->c - 3 * this->b * this->d;
        if (discriminant >= 0)
        {
            const double sqrt_discriminant = std::sqrt(discriminant);
            update_max_abs_value((-this->c + sqrt_discriminant) / (3 * this->d));
            update_max_abs_value((-this->c - sqrt_discriminant) / (3 * this->d));
        }
    }
    else if (this->c != 0)
    {
        update_max_abs_value(-this->b / (2 * this->c));
    }

    return max_abs_value;
}

std::set<double> CubicPoly::approximate_linear(double eps, double s_start, double s_end) const
{
    require_or_throw(std::isfinite(eps) && eps > 0, "eps must be finite and greater than 0 (got {})", eps);

    if (s_start == s_end)
        return {};

    if (d == 0 && c == 0) // linear case: /
        return {s_start, s_end};

    std::vector<double> s_samples;
    if (d == 0 && c != 0) // quadratic case: U
    {
        const double step = 2.0 * std::sqrt(std::abs(eps / c));
        for (double s = s_start; s < s_end; s += step)
            s_samples.push_back(s);
    }
    else // cubic case
    {
        // transform to parametric form
        const double d_p =
            -d * s_start * s_start * s_start + d * s_end * s_end * s_end - 3 * d * s_start * s_end * s_end + 3 * d * s_start * s_start * s_end;
        const double c_p = 3 * d * s_start * s_start * s_start + 3 * d * s_start * s_end * s_end - 6 * d * s_start * s_start * s_end +
                           c * s_start * s_start + c * s_end * s_end - 2 * c * s_start * s_end;
        const double b_p = -3 * d * s_start * s_start * s_start + 3 * d * s_start * s_start * s_end - 2 * c * s_start * s_start +
                           2 * c * s_start * s_end - b * s_start + b * s_end;
        const double a_p = d * s_start * s_start * s_start + c * s_start * s_start + b * s_start + a;

        const std::array<Vec1D, 4> coefficients = {{{a_p}, {b_p}, {c_p}, {d_p}}};
        const std::set<double>     p_vals = CubicBezier1D(CubicBezier1D::get_control_points(coefficients)).approximate_linear(eps);

        s_samples.push_back(s_start);
        for (const double p : p_vals)
            s_samples.push_back(p * (s_end - s_start) + s_start);
    }

    if ((s_end - s_samples.back()) < 1e-9 && (s_samples.size() != 1))
        s_samples.back() = s_end;
    else
        s_samples.push_back(s_end);

    std::set<double> s_sample_set(s_samples.begin(), s_samples.end());
    return s_sample_set;
}

void CubicPoly::negate()
{
    a = -a;
    b = -b;
    c = -c;
    d = -d;
}

bool CubicPoly::is_zero() const
{
    return (a == 0) && (b == 0) && (c == 0) && (d == 0);
}

void CubicPoly::set_zero()
{
    a = 0;
    b = 0;
    c = 0;
    d = 0;
}

std::optional<double> CubicProfile::evaluate(double s) const
{
    const std::optional<CubicPoly>& poly = this->get_poly(s);
    if (!poly)
        return std::nullopt;
    return poly->evaluate(s);
}

std::optional<double> CubicProfile::derivative(double s) const
{
    const std::optional<CubicPoly>& poly = this->get_poly(s);
    if (!poly)
        return std::nullopt;
    return poly->derivative(s);
}

CubicProfile CubicProfile::negate() const
{
    CubicProfile negated = *this;
    for (auto& s_poly : negated.s_to_poly)
        s_poly.second.negate();
    return negated;
}

CubicProfile CubicProfile::add(const CubicProfile& other) const
{
    if (other.s_to_poly.empty())
        return *this;
    if (this->s_to_poly.empty())
        return other;

    std::set<double> s_values = get_map_keys(this->s_to_poly);
    std::set<double> s_values_other = get_map_keys(other.s_to_poly);
    s_values.insert(s_values_other.begin(), s_values_other.end());

    CubicProfile retval;
    for (const double s : s_values)
    {
        const std::optional<CubicPoly>& this_poly = this->get_poly(s);
        const std::optional<CubicPoly>& other_poly = other.get_poly(s);

        if (!this_poly || !other_poly) // can't be both invalid
        {
            retval.s_to_poly[s] = this_poly.has_value() ? *this_poly : *other_poly;
            continue;
        }

        CubicPoly res;
        res.a = this_poly->a + other_poly->a;
        res.b = this_poly->b + other_poly->b;
        res.c = this_poly->c + other_poly->c;
        res.d = this_poly->d + other_poly->d;
        retval.s_to_poly[s] = res;
    }
    return retval;
}

std::optional<CubicPoly> CubicProfile::get_poly(double s) const
{
    if (this->s_to_poly.empty())
        return std::nullopt;

    if (s < this->s_to_poly.begin()->first)
        return std::nullopt;

    // will return last poly for s > s_end
    auto target_poly_iter = this->s_to_poly.upper_bound(s);
    if (target_poly_iter != this->s_to_poly.begin())
        target_poly_iter--;
    return target_poly_iter->second;
}

double CubicProfile::max_abs_value(double s_start, double s_end) const
{
    if ((s_start == s_end) || this->s_to_poly.empty())
        return 0;

    auto poly_end_iter = this->s_to_poly.lower_bound(s_end);
    auto poly_start_iter = this->s_to_poly.upper_bound(s_start);
    if (poly_start_iter != this->s_to_poly.begin())
        poly_start_iter--;

    double max_abs_value = 0;
    for (auto poly_iter = poly_start_iter; poly_iter != poly_end_iter; poly_iter++)
    {
        const double s_start_poly = std::max(poly_iter->first, s_start);
        const double s_end_poly = (std::next(poly_iter) == poly_end_iter) ? s_end : std::min(std::next(poly_iter)->first, s_end);
        max_abs_value = std::max(max_abs_value, poly_iter->second.max_abs_value(s_start_poly, s_end_poly));
    }

    return max_abs_value;
}

std::set<double> CubicProfile::approximate_linear(double eps, double s_start, double s_end) const
{
    if ((s_start == s_end) || this->s_to_poly.empty())
        return {};

    auto poly_end_iter = this->s_to_poly.lower_bound(s_end);
    auto poly_start_iter = this->s_to_poly.upper_bound(s_start);
    if (poly_start_iter != this->s_to_poly.begin())
        poly_start_iter--;

    std::set<double> s_samples;
    for (auto poly_iter = poly_start_iter; poly_iter != poly_end_iter; poly_iter++)
    {
        const double s_start_poly = std::max(poly_iter->first, s_start);
        const double s_end_poly = (std::next(poly_iter) == poly_end_iter) ? s_end : std::min(std::next(poly_iter)->first, s_end);

        std::set<double> s_samples_poly = poly_iter->second.approximate_linear(eps, s_start_poly, s_end_poly);
        if (s_samples_poly.size() < 2)
        {
            std::string err_msg = std::string("expected at least two sample points, got ") + std::to_string(s_samples_poly.size()) +
                                  std::string(" for [") + std::to_string(s_start_poly) + ' ' + std::to_string(s_end_poly) + ']';
            throw std::runtime_error(err_msg);
        }

        s_samples.insert(s_samples_poly.begin(), s_samples_poly.end());
    }

    return s_samples;
}

} // namespace odr
