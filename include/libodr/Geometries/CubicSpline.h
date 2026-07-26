#pragma once
#include <cstddef>
#include <map>
#include <optional>
#include <set>

namespace odr
{

struct CubicPoly
{
    CubicPoly() = default;
    CubicPoly(double a, double b, double c, double d, double s_origin = 0.0); // constructs a global cubic from local coefficients in (s - s_origin)

    double evaluate(double s) const;
    double derivative(double s) const;
    double max_value(double s_start, double s_end) const;

    void negate();
    bool is_zero() const;
    void set_zero();

    std::set<double> approximate_linear(double eps, double s_start, double s_end) const;

    double a = 0;
    double b = 0;
    double c = 0;
    double d = 0;
};

struct CubicProfile
{
    CubicProfile() = default;
    virtual ~CubicProfile() = default;

    std::optional<double> evaluate(double s) const;
    std::optional<double> derivative(double s) const;

    double max_value(double s_start, double s_end) const;

    std::optional<CubicPoly> get_poly(double s) const;

    [[nodiscard]] CubicProfile negate() const;
    [[nodiscard]] CubicProfile add(const CubicProfile& other) const;

    std::set<double> approximate_linear(double eps, double s_start, double s_end) const;

    std::map<double /* s0 */, CubicPoly> segments;
};

} // namespace odr