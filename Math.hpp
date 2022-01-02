#pragma once

#include <algorithm>
#include <array>
#define _USE_MATH_DEFINES
#include <cmath>
#include <numeric>
#include <type_traits>
#include <vector>

#include <glm/glm.hpp>

namespace odr
{
template<typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
using Vec = glm::vec<Dim, T>;

template<typename T, size_t Dim, typename std::enable_if_t<(Dim > 1)>* = nullptr, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
using Mat = glm::mat<Dim, Dim, T>;

using Vec1D = Vec<double, 1>;
using Vec2D = Vec<double, 2>;
using Vec3D = Vec<double, 3>;
using Mat3D = Mat<double, 3>;
using Line3D = std::vector<Vec3D>;

template<typename T>
int sign(T val)
{
    return (T(0) < val) - (val < T(0));
}

template<typename T, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
constexpr Mat<T, 3> EulerAnglesToMatrix(T r_x, T r_y, T r_z)
{
    /* precompute sines and cosines of Euler angles */
    const T su = std::sin(r_x);
    const T cu = std::cos(r_x);
    const T sv = std::sin(r_y);
    const T cv = std::cos(r_y);
    const T sw = std::sin(r_z);
    const T cw = std::cos(r_z);

    /* create and populate RotationMatrix */
    Mat<T, 3> RotMat;
    RotMat[0][0] = cv * cw;
    RotMat[0][1] = su * sv * cw - cu * sw;
    RotMat[0][2] = su * sw + cu * sv * cw;
    RotMat[1][0] = cv * sw;
    RotMat[1][1] = cu * cw + su * sv * sw;
    RotMat[1][2] = cu * sv * sw - su * cw;
    RotMat[2][0] = -sv;
    RotMat[2][1] = su * cv;
    RotMat[2][2] = cu * cv;

    return RotMat;
}

} // namespace odr