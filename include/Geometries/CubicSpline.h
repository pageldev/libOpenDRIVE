#pragma once
#include <cstddef>
#include <map>
#include <set>

namespace odr
{

struct Poly3
{
    Poly3() = default;
    Poly3(double s0, double a, double b, double c, double d);

    double get(double s) const;
    double get_grad(double s) const;
    double get_max(double s_start, double s_end) const;
    void   negate();

    std::set<double> approximate_linear(double eps, double s_start, double s_end) const;

    double a = 0, b = 0, c = 0, d = 0;
};

struct CubicSpline
{
    CubicSpline() = default;

    std::size_t size() const;
    double      get(double s) const;
    double      get_grad(double s) const;
    double      get_max(double s_start, double s_end) const;
    CubicSpline negate() const;
    CubicSpline add(const CubicSpline& other) const;
    Poly3       get_poly(double s) const;

    std::set<double> approximate_linear(double eps, double s_start, double s_end) const;

    std::map<double, Poly3> s0_to_poly;
};

} // namespace odr