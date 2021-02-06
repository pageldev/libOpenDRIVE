#pragma once

#include <map>
#include <memory>
#include <set>
#include <stddef.h>

namespace odr
{
struct Poly3
{
    Poly3() = default;
    Poly3(double s0, double a, double b, double c, double d);
    virtual ~Poly3() = default;

    double get(double s) const;
    double get_grad(double s) const;
    double get_max(double s_start, double s_end) const;

    std::set<double> approximate_linear(double eps, double s_start, double s_end) const;

    void negate();

    double a = 0, b = 0, c = 0, d = 0;
};

struct CubicSpline
{
    CubicSpline() = default;
    virtual ~CubicSpline() = default;

    size_t size() const;
    double get(double s) const;
    double get_grad(double s) const;

    CubicSpline negate() const;
    CubicSpline add(const CubicSpline& other) const;
    Poly3       get_poly(double s) const;
    double      get_max(double s_start, double s_end) const;

    std::set<double> approximate_linear(double eps, double s_start, double s_end) const;

    std::map<double, Poly3> s0_to_poly;
};

} // namespace odr