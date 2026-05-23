#include "RefLine.h"
#include "Geometries/Arc.h"
#include "Geometries/Line.h"
#include "Geometries/Spiral.h"
#include "Math.hpp"
#include "Utils.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <memory>

namespace odr
{

TEST_CASE("RefLine match on straight line", "[match]")
{
    const double length = 100.0;
    const double x0 = 10.0;
    const double y0 = -20.0;
    const double hdg0 = 0.5; // ~28.6 degrees

    RefLine ref_line(length);
    ref_line.s0_to_geometry[0.0] = std::make_unique<Line>(0.0, x0, y0, hdg0, length);

    // 1. Test points exactly on the line
    for (double test_s = 0.0; test_s <= length; test_s += 20.0)
    {
        const Vec3D  pt = ref_line.get_xyz(test_s);
        const double matched_s = ref_line.match(pt[0], pt[1]);
        REQUIRE(std::abs(matched_s - test_s) < 1e-2);
    }

    // 2. Test points offset to the side of the line
    const double side_offset = 5.0;
    const double dx = -std::sin(hdg0) * side_offset;
    const double dy = std::cos(hdg0) * side_offset;

    for (double test_s = 10.0; test_s <= length - 10.0; test_s += 20.0)
    {
        const Vec3D  pt_on_line = ref_line.get_xyz(test_s);
        const double matched_s = ref_line.match(pt_on_line[0] + dx, pt_on_line[1] + dy);
        REQUIRE(std::abs(matched_s - test_s) < 1e-2);
    }

    // 3. Test boundary clamping (out-of-bounds points)
    // Point before the start of the line (s = -10.0)
    const Vec3D  pt_before = ref_line.get_xyz(-10.0);
    const double matched_before_s = ref_line.match(pt_before[0], pt_before[1]);
    REQUIRE(std::abs(matched_before_s - 0.0) < 1e-2);

    // Point after the end of the line (s = 110.0)
    const Vec3D  pt_after = ref_line.get_xyz(110.0);
    const double matched_after_s = ref_line.match(pt_after[0], pt_after[1]);
    REQUIRE(std::abs(matched_after_s - length) < 1e-2);
}

TEST_CASE("RefLine match on circle loop", "[match]")
{
    const double radius = 50.0;
    const double length = 2.0 * 3.141592653589793 * radius;

    RefLine ref_line(length);
    ref_line.s0_to_geometry[0.0] = std::make_unique<Arc>(0.0, 0.0, 0.0, 0.0, length, 1.0 / radius);

    // Test matching for various points along the circle
    for (double test_s = 5.0; test_s < length; test_s += 25.0)
    {
        const Vec3D  pt = ref_line.get_xyz(test_s);
        const double matched_s = ref_line.match(pt[0], pt[1]);
        REQUIRE(std::abs(matched_s - test_s) < 0.05);
    }
}

TEST_CASE("RefLine match on spiral", "[match]")
{
    const double length = 50.0;
    const double curv_start = 0.0;
    const double curv_end = 0.05;

    RefLine ref_line(length);
    ref_line.s0_to_geometry[0.0] = std::make_unique<Spiral>(0.0, 0.0, 0.0, 0.0, length, curv_start, curv_end);

    // Test matching along the spiral
    for (double test_s = 5.0; test_s < length; test_s += 10.0)
    {
        const Vec3D  pt = ref_line.get_xyz(test_s);
        const double matched_s = ref_line.match(pt[0], pt[1]);
        REQUIRE(std::abs(matched_s - test_s) < 0.05);
    }
}

TEST_CASE("RefLine match on P-loop curve", "[match]")
{
    // A P-loop consisting of a 100m straight line, followed by a circular loop (radius 30m)
    // Curvature is -1/30 (clockwise turning), and length is 2 * pi * 30 ≈ 188.495559
    const double line_len = 100.0;
    const double arc_len = 2.0 * 3.141592653589793 * 30.0;
    const double total_len = line_len + arc_len;

    RefLine ref_line(total_len);
    ref_line.s0_to_geometry[0.0] = std::make_unique<Line>(0.0, 0.0, 0.0, 0.0, line_len);
    ref_line.s0_to_geometry[line_len] = std::make_unique<Arc>(line_len, line_len, 0.0, 0.0, arc_len, -1.0 / 30.0);
    // Target point P on the loop at s = 250.0
    const double test_s = 250.0;
    const Vec3D  pt = ref_line.get_xyz(test_s);
    const double matched_s = ref_line.match(pt[0], pt[1]);
    REQUIRE(std::abs(matched_s - test_s) < 0.05);
}

} // namespace odr
