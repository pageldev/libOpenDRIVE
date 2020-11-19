#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <set>
#include <type_traits>
#include <vector>
#include <array>
#include <string>

namespace odr
{
template<typename T, size_t Dim, typename std::enable_if_t<(Dim > 1)>* = nullptr, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
using Vec = std::array<T, Dim>;

using Vec2D = Vec<double, 2>;
using Vec3D = Vec<double, 3>;

template<typename T, size_t Dim, typename std::enable_if_t<(Dim > 1)>* = nullptr, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
using Mat = std::array<std::array<T, Dim>, Dim>;

using Mat3D = Mat<double, 3>;

struct Box2D
{
    Box2D();
    Box2D(Vec2D min, Vec2D max);
    double get_distance(const Vec2D& pt);

    Vec2D  min, max;
    Vec2D  center;
    double width, height;
};

template<typename T>
int sign(T val)
{
    return (T(0) < val) - (val < T(0));
}

template<typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
constexpr T euclDistance(const Vec<T, Dim> a, const Vec<T, Dim> b)
{
    return std::inner_product(a.begin(), a.end(), b.begin(), T(0), std::plus<T>(), [](T a, T b) {
        T c = b - a;
        return c * c;
    });
}

template<typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
constexpr T squaredNorm(const Vec<T, Dim> v)
{
    return std::inner_product(v.begin(), v.end(), v.begin(), T(0));
}

template<typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
constexpr T norm(const Vec<T, Dim> v)
{
    return std::sqrt(squaredNorm<T, Dim>(v));
}

template<typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
constexpr Vec<T, Dim> normalize(const Vec<T, Dim> v)
{
    Vec<T, Dim> e_v;
    const T     n = norm(v);
    std::transform(v.begin(), v.end(), e_v.begin(), [&](const T& a) { return a / n; });
    return e_v;
}

template<typename T, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
constexpr Vec<T, 3> crossProduct(const Vec<T, 3> a, const Vec<T, 3> b)
{
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

template<typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
constexpr Vec<T, Dim> MatVecMultiplication(const Mat<T, Dim> m, const Vec<T, Dim> v)
{
    Vec<T, Dim> res;
    res.fill(T{0});
    for (size_t idx = 0; idx < Dim * Dim; idx++)
        res[idx / Dim] += ((double*)m.data())[idx] * v[idx % Dim];
    return res;
}

template<class C, class T, T C::*member>
struct SharedPtrCmp
{
    bool operator()(const std::shared_ptr<C>& lhs, const std::shared_ptr<C>& rhs) const { return (*lhs).*member < (*rhs).*member; }
};

template<typename T, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
Box2D get_bbox_for_s_values(const std::vector<T>& s_values, const std::function<Vec2D(T)>& get_xyz)
{
    std::vector<Vec2D> points;
    points.reserve(s_values.size());
    for (const T& s_val : s_values)
        points.push_back(get_xyz(s_val));

    auto iter_min_max_x = std::minmax_element(points.begin(), points.end(), [](const Vec2D& lhs, const Vec2D& rhs) { return lhs[0] < rhs[0]; });
    auto iter_min_max_y = std::minmax_element(points.begin(), points.end(), [](const Vec2D& lhs, const Vec2D& rhs) { return lhs[1] < rhs[1]; });

    Vec2D min = {iter_min_max_x.first->at(0), iter_min_max_y.first->at(1)};
    Vec2D max = {iter_min_max_x.second->at(0), iter_min_max_y.second->at(1)};

    return Box2D(min, max);
};

template<typename T, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
T golden_section_search(const std::function<T(T)>& f, T a, T b, const T tol)
{
    const T invphi = (std::sqrt(5) - 1) / 2;
    const T invphi2 = (3 - std::sqrt(5)) / 2;

    T h = b - a;
    if (h <= tol)
        return 0.5 * (a + b);

    // Required steps to achieve tolerance
    int n = static_cast<int>(std::ceil(std::log(tol / h) / std::log(invphi)));

    T c = a + invphi2 * h;
    T d = a + invphi * h;
    T yc = f(c);
    T yd = f(d);

    for (int k = 0; k < (n - 1); k++)
    {
        if (yc < yd)
        {
            b = d;
            d = c;
            yd = yc;
            h = invphi * h;
            c = a + invphi2 * h;
            yc = f(c);
        }
        else
        {
            a = c;
            c = d;
            yc = yd;
            h = invphi * h;
            d = a + invphi * h;
            yd = f(d);
        }
    }

    if (yc < yd)
        return 0.5 * (a + d);

    return 0.5 * (c + b);
}

} // namespace odr