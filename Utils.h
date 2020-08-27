#pragma once
#include "Geometries/Geometries.h"

#include <vector>

template <typename T>
int sign(T val)
{
    return (T(0) < val) - (val < T(0));
}

std::vector<Point3D> rdp(const std::vector<Point3D> &points, const double epsilon = 0.1);
