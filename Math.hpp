#pragma once

#include <algorithm>
#include <array>
#define _USE_MATH_DEFINES
#include <cmath>
#include <numeric>
#include <type_traits>
#include <vector>

namespace odr
{
template<typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
using Vec = std::array<T, Dim>;

using Vec1D = Vec<double, 1>;
using Vec2D = Vec<double, 2>;
using Vec3D = Vec<double, 3>;
using Line3D = std::vector<Vec3D>;

template<typename T, size_t Dim, typename std::enable_if_t<(Dim > 1)>* = nullptr, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
using Mat = std::array<std::array<T, Dim>, Dim>;

using Mat3D = Mat<double, 3>;

template<typename T>
int sign(T val)
{
    return (T(0) < val) - (val < T(0));
}

template<typename T, size_t Dim, typename BinaryOperation, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
constexpr Vec<T, Dim> operation(const Vec<T, Dim>& a, const Vec<T, Dim>& b, BinaryOperation op)
{
    Vec<T, Dim> res;
    for (size_t idx = 0; idx < Dim; idx++)
        res[idx] = op(a[idx], b[idx]);
    return res;
}

template<typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
constexpr Vec<T, Dim> add(const Vec<T, Dim>& a, const Vec<T, Dim>& b)
{
    return operation<T, Dim, std::plus<T>>(a, b, std::plus<T>());
}

template<typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
constexpr Vec<T, Dim> sub(const Vec<T, Dim>& a, const Vec<T, Dim>& b)
{
    return operation<T, Dim, std::minus<T>>(a, b, std::minus<T>());
}

template<typename T, size_t Dim, typename BinaryOperation, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
constexpr Vec<T, Dim> operation(const T& scalar, const Vec<T, Dim>& a, BinaryOperation op)
{
    Vec<T, Dim> res;
    for (size_t idx = 0; idx < Dim; idx++)
        res[idx] = op(scalar, a[idx]);
    return res;
}

template<typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
constexpr Vec<T, Dim> add(const T& scalar, const Vec<T, Dim>& a)
{
    return operation<T, Dim, std::plus<T>>(scalar, a, std::plus<T>());
}

template<typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
constexpr Vec<T, Dim> sub(const T& scalar, const Vec<T, Dim>& a)
{
    return operation<T, Dim, std::minus<T>>(scalar, a, std::minus<T>());
}

template<typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
constexpr Vec<T, Dim> mut(const T& scalar, const Vec<T, Dim>& a)
{
    return operation<T, Dim, std::multiplies<T>>(scalar, a, std::multiplies<T>());
}

template<typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
constexpr T euclDistance(const Vec<T, Dim> a, const Vec<T, Dim> b)
{
    return std::sqrt(std::inner_product(a.begin(),
                                        a.end(),
                                        b.begin(),
                                        T(0),
                                        std::plus<T>(),
                                        [](T a, T b)
                                        {
                                            T c = b - a;
                                            return c * c;
                                        }));
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

} // namespace odr