#include "libodr/Geometries/Arc.h"
#include "libodr/Geometries/Line.h"
#include "libodr/Lane.h"
#include "libodr/LaneSection.h"
#include "libodr/OpenDriveMap.h"
#include "libodr/RefLine.h"
#include "libodr/Road.h"
#include "libodr/RoadMark.h"
#include "libodr/RoadObject.h"
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

struct OpenDriveFixture
{
    OpenDriveFixture()
    {
        pugi::xml_parse_result result = this->xml_doc.load_file("test.xodr");
        REQUIRE(result);

        this->odr_map = std::make_unique<odr::OpenDriveMap>();
        const odr::XodrParseResult parse_result = this->odr_map->load(xml_doc);
        REQUIRE(parse_result.errors.empty());
        REQUIRE(!(this->odr_map->id_to_road.empty()));
    }

    pugi::xml_document                 xml_doc;
    std::unique_ptr<odr::OpenDriveMap> odr_map;
};

TEST_CASE("odr types are movable", "[types]")
{
    STATIC_REQUIRE(std::is_move_constructible_v<odr::OpenDriveMap>);
    STATIC_REQUIRE(std::is_move_assignable_v<odr::OpenDriveMap>);
    STATIC_REQUIRE(std::is_move_constructible_v<odr::Road>);
    STATIC_REQUIRE(std::is_move_assignable_v<odr::Road>);
    STATIC_REQUIRE(std::is_move_constructible_v<odr::RoadObject>);
    STATIC_REQUIRE(std::is_move_assignable_v<odr::RoadObject>);
    STATIC_REQUIRE(std::is_move_constructible_v<odr::RoadObjectRepeat>);
    STATIC_REQUIRE(std::is_move_assignable_v<odr::RoadObjectRepeat>);
    STATIC_REQUIRE(std::is_move_constructible_v<odr::RoadObjectOutline>);
    STATIC_REQUIRE(std::is_move_assignable_v<odr::RoadObjectOutline>);
    STATIC_REQUIRE(std::is_move_constructible_v<odr::LaneSection>);
    STATIC_REQUIRE(std::is_move_assignable_v<odr::LaneSection>);
    STATIC_REQUIRE(std::is_move_constructible_v<odr::Lane>);
    STATIC_REQUIRE(std::is_move_assignable_v<odr::Lane>);
    STATIC_REQUIRE(std::is_move_constructible_v<odr::Junction>);
    STATIC_REQUIRE(std::is_move_assignable_v<odr::Junction>);
    STATIC_REQUIRE(std::is_move_constructible_v<odr::JunctionConnection>);
    STATIC_REQUIRE(std::is_move_assignable_v<odr::JunctionConnection>);
}

TEST_CASE("RefLine match on a long straight line", "[match]")
{
    struct CountingLine : odr::Line
    {
        using odr::Line::Line;
        mutable std::size_t evaluations = 0;

        odr::Vec2D get_xy(double s) const override
        {
            ++evaluations;
            return odr::Line::get_xy(s);
        }
    };

    odr::RefLine ref_line(10000.0);
    auto         line = std::make_unique<CountingLine>(0.0, 0.0, 0.0, 0.0, ref_line.length);
    const auto*  counted_line = line.get();
    ref_line.s_to_geometry[0.0] = std::move(line);

    REQUIRE(std::abs(ref_line.match(5000.37, 4.0) - 5000.37) < 1e-2);
    REQUIRE(counted_line->evaluations < 100);
    REQUIRE(std::abs(ref_line.match(-5.0, 0.0)) < 1e-2);
    REQUIRE(std::abs(ref_line.match(10005.0, 0.0) - ref_line.length) < 1e-2);
}

TEST_CASE("RefLine match across line and arc", "[match]")
{
    const double arc_length = 2.0 * std::acos(-1.0) * 30.0;
    odr::RefLine ref_line(100.0 + arc_length);
    ref_line.s_to_geometry[0.0] = std::make_unique<odr::Line>(0.0, 0.0, 0.0, 0.0, 100.0);
    ref_line.s_to_geometry[100.0] = std::make_unique<odr::Arc>(100.0, 100.0, 0.0, 0.0, arc_length, -1.0 / 30.0);

    const odr::Vec3D point = ref_line.get_xyz(250.0);
    REQUIRE(std::abs(ref_line.match(point[0], point[1]) - 250.0) < 1e-2);
}

TEST_CASE_METHOD(OpenDriveFixture, "Basic OpenDriveMap check", "[xodr]")
{
    // basic routing test
    auto graph = odr_map->get_routing_graph();
    auto path = graph.shortest_path(odr::LaneKey("43", 0.0, 1), odr::LaneKey("41", 0.0, 1));
    REQUIRE(path.size() == 15);

    // road sanity
    for (const auto& [_, road] : odr_map->id_to_road)
    {
        INFO("road: " << road.id << ", length: " << road.length);
        REQUIRE(road.length >= 0.0);
        REQUIRE(!road.s_to_lane_section.empty());
        for (const auto& [_, ls] : road.s_to_lane_section)
        {
            const double s_start = ls.s;
            const double s_end = ls.get_end();
            REQUIRE(s_start >= 0.0);
            REQUIRE(s_end > s_start);
            for (const auto& [_, lane] : ls.id_to_lane)
            {
                std::vector<odr::SingleRoadMark> roadmarks = lane.get_roadmarks(s_start, s_end);
                (void)roadmarks; // silence unused var if not checked
            }
        }
    }
}

TEST_CASE_METHOD(OpenDriveFixture, "Routing check", "[xodr]")
{
    const std::vector<odr::RoutingPath> expected_paths = {
        {{"37", 0, 1}, {"215", 0, -1}, {"47", 0, -1}},
        {{"24", 0, -1}, {"447", 0, -1}, {"20", 0, -1}, {"765", 0, -1}, {"21", 0, -1}, {"39", 0, -1}},
        {{"17", 0, -1},
         {"18", 0, -1},
         {"747", 0, -1},
         {"747", 1.7623724829973639, -1},
         {"41", 0, 1},
         {"388", 0, -1},
         {"35", 0, 1},
         {"480", 0, -1},
         {"480", 20.515904612542592, -1},
         {"480", 20.858538341635004, -1},
         {"53", 0, 1},
         {"817", 0, -1},
         {"63", 0, -4},
         {"64", 0, -4},
         {"65", 0, -4}}};

    const odr::RoutingGraph graph = odr_map->get_routing_graph();
    for (const odr::RoutingPath& expected_path : expected_paths)
    {
        const odr::RoutingPath path = graph.shortest_path(expected_path.front(), expected_path.back());
        REQUIRE(path == expected_path);
    }
}
