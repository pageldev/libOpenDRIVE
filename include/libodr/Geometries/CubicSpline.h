#pragma once
#include <cstddef>
#include <map>
#include <optional>

namespace odr
{

struct CubicBounds
{
    double min = 0;
    double max = 0;
    double d1_min = 0; // signed
    double d1_max = 0;
    double d1 = 0;
    double d2 = 0;
    double d3 = 0;
};

struct CubicPoly
{
    CubicPoly() = default;
    CubicPoly(double a, double b, double c, double d, double s_origin = 0.0); // constructs a global cubic from local coefficients in (s - s_origin)

    double      evaluate(double s) const;
    double      derivative(double s) const;
    CubicBounds bounds(double s_start, double s_end) const;

    void add(const CubicPoly& other);
    void subtract(const CubicPoly& other);
    void negate();
    bool is_zero() const;
    void set_zero();

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

    std::optional<CubicPoly> get_poly(double s) const;

    [[nodiscard]] CubicProfile negate() const;
    [[nodiscard]] CubicProfile add(const CubicProfile& other) const;

    std::map<double, CubicPoly> s_to_poly;
};

} // namespace odr
