#include "CubicSpline.h"

namespace odr
{
Poly3::Poly3(double s0, double a, double b, double c, double d) : s0(s0), a(a), b(b), c(c), d(d) {}

double Poly3::get(double s) const
{
    const double ds = s - s0;
    return a + b * ds + c * ds * ds + d * ds * ds * ds;
}

double Poly3::get_grad(double s) const { return b + 2 * c * s + 3 * d * s * s; }

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

std::shared_ptr<const Poly3> CubicSpline::get_poly(double s) const
{
    std::shared_ptr<const Poly3> poly = nullptr;
    if (this->s0_to_poly.size() > 0)
    {
        auto target_poly_iter = this->s0_to_poly.upper_bound(s);
        if (target_poly_iter != this->s0_to_poly.begin())
            target_poly_iter--;
        poly = target_poly_iter->second;
    }
    return poly;
}

} // namespace odr