#pragma once

#include "Geometries.h"

#include <vector>


std::vector<Point3D> rdp( const std::vector<Point3D>& points, double epsilon = 0.1 );
