#include "OpenDriveMap.h"
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Basic OpenDriveMap check", "[xodr]")
{
    odr::OpenDriveMap odr_map("test.xodr");
    REQUIRE(odr_map.xml_parse_result);
    REQUIRE(!odr_map.get_roads().empty());

    for (const odr::Road& road : odr_map.get_roads())
    {
        INFO("road: " << road.id << ", length: " << road.length);
        REQUIRE(road.length >= 0.0);
        REQUIRE(!road.s_to_lanesection.empty());
        for (const odr::LaneSection& lanesection : road.get_lanesections())
        {
            const double s_start = lanesection.s0;
            const double s_end = road.get_lanesection_end(lanesection);
            REQUIRE(s_start >= 0.0);
            REQUIRE(s_end > s_start);
            for (const odr::Lane& lane : lanesection.get_lanes())
            {
                std::vector<odr::RoadMark> roadmarks = lane.get_roadmarks(s_start, s_end);
            }
        }
    }
}