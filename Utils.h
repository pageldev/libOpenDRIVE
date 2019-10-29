#pragma once

#include "Geometries.h"

#include <vector>
#include <memory>


template <typename T>
class PtrCompareS0
{
    public:
        bool operator()(const std::shared_ptr<T>& lhs, const std::shared_ptr<T>& rhs) const
        {
            return (lhs->s0 < rhs->s0);
        }
};

std::vector<std::pair<double, double>> rdp( const std::vector<std::pair<double, double>>& points, double epsilon = 0.1 );

std::vector<Point3D> rdp( const std::vector<Point3D>& points, double epsilon = 0.1 );
