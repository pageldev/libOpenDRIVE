#include "libodr/Geometries/CubicSpline.h"
#include "libodr/Math.hpp"
#include "libodr/Utils.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <set>

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

CubicBounds CubicPoly::bounds(double s_start, double s_end) const
{
    CubicBounds out;

    const double val_start = evaluate(s_start);
    const double val_end = evaluate(s_end);
    out.min = std::min(val_start, val_end);
    out.max = std::max(val_start, val_end);

    const auto include_in_range = [&](double& lower, double& upper, double s, const auto& evaluate)
    {
        if (s > s_start && s < s_end)
        {
            const double value = evaluate(s);
            lower = std::min(lower, value);
            upper = std::max(upper, value);
        }
    };

    if (d != 0) // quadratic derivative
    {
        const double discriminant = c * c - 3 * b * d;
        if (discriminant >= 0) // internal extrema may exceed the endpoint bounds
        {
            const double root = std::sqrt(discriminant);
            include_in_range(out.min, out.max, (-c - root) / (3 * d), [&](double s) { return evaluate(s); }); // local max
            include_in_range(out.min, out.max, (-c + root) / (3 * d), [&](double s) { return evaluate(s); }); // local min
        }
    }
    else if (c != 0) // d == 0, c != 0 -> linear derivative
        include_in_range(out.min, out.max, -b / (2 * c), [&](double s) { return evaluate(s); });

    out.d1_min = std::min(derivative(s_start), derivative(s_end));
    out.d1_max = std::max(derivative(s_start), derivative(s_end));
    if (d != 0)
        include_in_range(out.d1_min, out.d1_max, -c / (3 * d), [&](double s) { return derivative(s); });

    out.d1 = std::max(std::abs(out.d1_min), std::abs(out.d1_max));
    out.d2 = std::max(std::abs(2 * c + 6 * d * s_start), std::abs(2 * c + 6 * d * s_end));
    out.d3 = std::abs(6 * d);

    return out;
}

void CubicPoly::add(const CubicPoly& other)
{
    a += other.a;
    b += other.b;
    c += other.c;
    d += other.d;
}

void CubicPoly::subtract(const CubicPoly& other)
{
    a -= other.a;
    b -= other.b;
    c -= other.c;
    d -= other.d;
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

        CubicPoly result = *this_poly;
        result.add(*other_poly);
        retval.s_to_poly[s] = result;
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

} // namespace odr
