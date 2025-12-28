#pragma once
#include <cstddef>
#include <map>
#include <set>

namespace odr
{

struct CubicPoly
{
    CubicPoly() = default;
    CubicPoly(double a, double b, double c, double d, double s_origin = 0.0); // constructs a global cubic from local coefficients in (s - s_origin)

    double evaluate(const double s) const;
    double derivative(const double s) const;
    double max_value(const double s_start, const double s_end) const;

    void negate();
    bool is_zero() const;
    void set_zero();
    bool isnan() const;

    std::set<double> approximate_linear(const double eps, const double s_start, const double s_end) const;

    double a = 0;
    double b = 0;
    double c = 0;
    double d = 0;
};

struct CubicProfile
{
    CubicProfile() = default;

    double evaluate(const double s, const double default_val = 0.0, const bool extend_start = true) const;
    double derivative(const double s, const double default_val = 0.0, const bool extend_start = true) const;
    double max_value(const double s_start, const double s_end) const;

    CubicPoly get_poly(const double s, const bool extend_start = true) const;

    [[nodiscard]] CubicProfile negate() const;
    [[nodiscard]] CubicProfile add(const CubicProfile& other) const;

    std::set<double> approximate_linear(double eps, double s_start, double s_end) const;

    std::map<double /* s0 */, CubicPoly> segments;
};

} // namespace odr