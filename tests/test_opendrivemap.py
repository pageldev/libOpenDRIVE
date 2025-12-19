import inspect

import pytest
from opendrive import OpenDriveMap, LaneKey

def test_basic_opendrivemap_check():
    odr_map = OpenDriveMap("tests/test.xodr", center_map=False)
    assert odr_map.xml_parse_result
    assert len(odr_map.get_roads()) > 0

    # test routing
    graph = odr_map.get_routing_graph()
    path = graph.shortest_path(LaneKey("43", 0.0, 1), LaneKey("41", 0.0, 1))
    assert len(path) == 15

    for road in odr_map.get_roads():
        print(f"road: {road.id}, length: {road.length}")
        assert road.length >= 0.0
        assert len(road.s_to_lanesection) > 0
        for lanesection in road.get_lanesections():
            s_start = lanesection.s0
            s_end = road.get_lanesection_end(lanesection)
            assert s_start >= 0.0
            assert s_end > s_start
            for lane in lanesection.get_lanes():
                roadmarks = lane.get_roadmarks(s_start, s_end)
