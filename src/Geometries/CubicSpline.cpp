#include "libodr/Geometries/CubicSpline.h"
#include "libodr/Math.hpp"
#include "libodr/Utils.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <set>

namespace odr
{

CubicPoly::CubicPoly(double a, double b, double c, double d, double s_origin) : a(a), b(b), c(c), d(d), s_origin(s_origin)
{
    require_or_throw(std::isfinite(a), "a must not be NaN");
    require_or_throw(std::isfinite(b), "b must not be NaN");
    require_or_throw(std::isfinite(c), "c must not be NaN");
    require_or_throw(std::isfinite(d), "d must not be NaN");
    require_or_throw(std::isfinite(s_origin), "s origin must not be NaN");
}

double CubicPoly::evaluate(double s) const
{
    const double ds = s - s_origin;
    return a + ds * (b + ds * (c + ds * d));
}

double CubicPoly::derivative(double s) const
{
    const double ds = s - s_origin;
    return b + ds * (2 * c + 3 * ds * d);
}

CubicBounds CubicPoly::bounds(double s_start, double s_end) const
{
    CubicBounds out;

    s_start -= s_origin;
    s_end -= s_origin;
    const auto evaluate_local = [&](double ds) { return a + ds * (b + ds * (c + ds * d)); };
    const auto derivative_local = [&](double ds) { return b + ds * (2 * c + 3 * ds * d); };

    const double val_start = evaluate_local(s_start);
    const double val_end = evaluate_local(s_end);
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
            include_in_range(out.min, out.max, (-c - root) / (3 * d), evaluate_local); // local max
            include_in_range(out.min, out.max, (-c + root) / (3 * d), evaluate_local); // local min
        }
    }
    else if (c != 0) // d == 0, c != 0 -> linear derivative
        include_in_range(out.min, out.max, -b / (2 * c), evaluate_local);

    out.d1_min = std::min(derivative_local(s_start), derivative_local(s_end));
    out.d1_max = std::max(derivative_local(s_start), derivative_local(s_end));
    if (d != 0)
        include_in_range(out.d1_min, out.d1_max, -c / (3 * d), derivative_local);

    out.d1 = std::max(std::abs(out.d1_min), std::abs(out.d1_max));
    out.d2 = std::max(std::abs(2 * c + 6 * d * s_start), std::abs(2 * c + 6 * d * s_end));
    out.d3 = std::abs(6 * d);

    return out;
}

void CubicPoly::rebase(double s_origin_new)
{
    a = evaluate(s_origin_new);
    b = derivative(s_origin_new);
    c += 3 * (s_origin_new - s_origin) * d;
    s_origin = s_origin_new;
}

void CubicPoly::add(const CubicPoly& other)
{
    const double new_origin = std::max(s_origin, other.s_origin);
    rebase(new_origin);
    CubicPoly other_rebased = other;
    other_rebased.rebase(new_origin);

    a += other_rebased.a;
    b += other_rebased.b;
    c += other_rebased.c;
    d += other_rebased.d;
}

void CubicPoly::subtract(const CubicPoly& other)
{
    const double new_origin = std::max(s_origin, other.s_origin);
    rebase(new_origin);
    CubicPoly other_rebased = other;
    other_rebased.rebase(new_origin);

    a -= other_rebased.a;
    b -= other_rebased.b;
    c -= other_rebased.c;
    d -= other_rebased.d;
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
