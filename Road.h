#pragma once

#include "Geometries/Geometries.h"
#include "RefLine.h"
#include "Utils.hpp"

#include <map>
#include <memory>

namespace odr
{

struct LaneSection;

struct LaneOffset;

class Road : public std::enable_shared_from_this<Road>
{
public:
    Road(double length, int id, int junction);

    Vec3D get_xyz(double s, double t, double z) const;
    Mat3D get_transformation_matrix(double s) const;

    int    id, junction;
    double length;

    std::shared_ptr<RefLine> ref_line;

    std::map<double, std::shared_ptr<LaneSection>> lane_sections;

    CubicSpline lane_offset;
    CubicSpline superelevation;
};

} // namespace odr