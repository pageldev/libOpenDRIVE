#pragma once
#include "Geometries/Geometries.h"

#include <vector>

namespace odr
{

template <typename T>
int sign(T val)
{
    return (T(0) < val) - (val < T(0));
}

std::vector<Point3D<double>> rdp(const std::vector<Point3D<double>> &points, const double epsilon = 0.1);

} // namespace odr