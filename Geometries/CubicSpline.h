#pragma once

#include <map>
#include <memory>
#include <stddef.h>

namespace odr
{
struct Poly3
{
    Poly3(double s0, double a, double b, double c, double d);
    double get(double s) const;
    double get_grad(double s) const;

    double s0, a, b, c, d;
};

struct CubicSpline
{
    CubicSpline() = default;
    virtual ~CubicSpline() = default;

    size_t size() const;
    double get(double s) const;
    double get_grad(double s) const;

    std::shared_ptr<const Poly3> get_poly(double s) const;

    std::map<double, std::shared_ptr<Poly3>> s0_to_poly;
};

} // namespace odr