#pragma once

#include "Geometries.h"

#include <cmath>
#include <utility>
#include <vector>
#include <stdexcept>


std::vector<std::pair<double, double>> rdp( const std::vector<std::pair<double, double>>& points, double epsilon = 0.1 );

std::vector<Point3D> rdp( const std::vector<Point3D>& points, double epsilon = 0.1 );
