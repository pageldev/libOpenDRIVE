#include "OpenDriveMap.h"
#include <catch2/catch_test_macros.hpp>

struct OpenDriveFixture
{
    OpenDriveFixture() : odr_map("test.xodr")
    {
        REQUIRE(!odr_map.get_roads().empty());
    }

    odr::OpenDriveMap odr_map;
};

TEST_CASE_METHOD(OpenDriveFixture, "Basic OpenDriveMap check", "[xodr]")
{
    // basic routing test
    auto graph = odr_map.get_routing_graph();
    auto path = graph.shortest_path(odr::LaneKey("43", 0.0, 1), odr::LaneKey("41", 0.0, 1));
    REQUIRE(path.size() == 15);

    // road sanity
    for (const odr::Road& road : odr_map.get_roads())
    {
        INFO("road: " << road.id << ", length: " << road.length);
        REQUIRE(road.length >= 0.0);
        REQUIRE(!road.s_to_lanesection.empty());
        for (const odr::LaneSection& ls : road.get_lanesections())
        {
            const double s_start = ls.s0;
            const double s_end = road.get_lanesection_end(ls);
            REQUIRE(s_start >= 0.0);
            REQUIRE(s_end > s_start);
            for (const odr::Lane& lane : ls.get_lanes())
            {
                std::vector<odr::RoadMark> roadmarks = lane.get_roadmarks(s_start, s_end);
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

    const odr::RoutingGraph graph = odr_map.get_routing_graph();
    for (const odr::RoutingPath& expected_path : expected_paths)
    {
        const odr::RoutingPath path = graph.shortest_path(expected_path.front(), expected_path.back());
        REQUIRE(path == expected_path);
    }
}

TEST_CASE_METHOD(OpenDriveFixture, "RoadNetworkMesh generation", "[mesh]")
{
    const odr::RoadNetworkMesh mesh = odr_map.get_road_network_mesh(0.1);

    SECTION("lanes mesh has vertices")
    {
        REQUIRE(!mesh.lanes_mesh.vertices.empty());
        REQUIRE(!mesh.lanes_mesh.indices.empty());
    }

    SECTION("lane_start_indices are valid")
    {
        for (const auto& [idx, lane_id] : mesh.lanes_mesh.lane_start_indices)
        {
            REQUIRE(idx < mesh.lanes_mesh.vertices.size());
        }
    }

    SECTION("road_start_indices are valid")
    {
        for (const auto& [idx, road_id] : mesh.lanes_mesh.road_start_indices)
        {
            REQUIRE(idx < mesh.lanes_mesh.vertices.size());
            REQUIRE(odr_map.id_to_road.count(road_id) > 0);
        }
    }

    SECTION("lane type lookup works for all mesh chunks")
    {
        for (const auto& [vert_idx, lane_id] : mesh.lanes_mesh.lane_start_indices)
        {
            const std::string road_id = mesh.lanes_mesh.get_road_id(vert_idx);
            const double      s0 = mesh.lanes_mesh.get_lanesec_s0(vert_idx);

            auto road_it = odr_map.id_to_road.find(road_id);
            REQUIRE(road_it != odr_map.id_to_road.end());

            const auto& lanesecs = road_it->second.s_to_lanesection;
            auto        ls_it = lanesecs.lower_bound(s0 - 1e-6);
            REQUIRE(ls_it != lanesecs.end());

            auto lane_it = ls_it->second.id_to_lane.find(lane_id);
            REQUIRE(lane_it != ls_it->second.id_to_lane.end());
            REQUIRE(!lane_it->second.type.empty());
        }
    }

    SECTION("junction road identification")
    {
        std::vector<std::string> junction_roads;
        for (const auto& [id, road] : odr_map.id_to_road)
        {
            if (!road.junction.empty() && road.junction != "-1")
            {
                junction_roads.push_back(id);
            }
        }
        // test.xodr has junctions (known from routing test)
        REQUIRE(!junction_roads.empty());
    }

    SECTION("lane outline indices are valid line pairs")
    {
        auto outline = mesh.lanes_mesh.get_lane_outline_indices();
        // outline indices must be even count (LINE_LIST = pairs)
        REQUIRE(outline.size() % 2 == 0);
        REQUIRE(!outline.empty());
        for (std::size_t i = 0; i < outline.size(); ++i)
        {
            REQUIRE(outline[i] < mesh.lanes_mesh.vertices.size());
        }
    }
}
