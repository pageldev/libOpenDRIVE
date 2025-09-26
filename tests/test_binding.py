import pytest
import os
from opendrive import OpenDriveMap, RoadNetworkMesh, LanesMesh, RoadmarksMesh, RoadObjectsMesh, RoadSignalsMesh, RoutingGraph, Mesh3D

# Fixture to load the OpenDriveMap
@pytest.fixture
def odr_map():
    file_path = "tests/test.xodr"
    if not os.path.exists(file_path):
        pytest.skip(f"File '{file_path}' not found in {os.getcwd()}")
    try:
        map_instance = OpenDriveMap(file_path, center_map=False)
        return map_instance
    except Exception as e:
        pytest.fail(f"Failed to load OpenDriveMap: {e}")

# Fixture to get the road network mesh
@pytest.fixture
def road_network_mesh(odr_map):
    eps = 0.1
    try:
        return odr_map.get_road_network_mesh(eps)
    except Exception as e:
        pytest.fail(f"Failed to get road network mesh: {e}")

def test_opendrive_map_attributes(odr_map):
    """Test the attributes of OpenDriveMap."""
    assert isinstance(odr_map.xodr_file, str), "xodr_file is not a string"
    assert isinstance(odr_map.proj4, str), "proj4 is not a string"
    assert isinstance(odr_map.x_offs, float), "x_offs is not a float"
    assert isinstance(odr_map.y_offs, float), "y_offs is not a float"

def test_roads(odr_map):
    """Test the roads retrieved from OpenDriveMap."""
    roads = odr_map.get_roads()
    assert isinstance(roads, list), "Roads is not a list"
    if roads:
        assert isinstance(odr_map.id_to_road, dict), "id_to_road is not a dict"
        print(f"Found {len(roads)} roads")

def test_junctions(odr_map):
    """Test the junctions retrieved from OpenDriveMap."""
    junctions = odr_map.get_junctions()
    assert isinstance(junctions, list), "Junctions is not a list"
    if junctions:
        assert isinstance(odr_map.id_to_junction, dict), "id_to_junction is not a dict"
        print(f"Found {len(junctions)} junctions")

def test_road_network_mesh(road_network_mesh):
    """Test the road network mesh."""
    assert isinstance(road_network_mesh, RoadNetworkMesh), "road_network_mesh is not a RoadNetworkMesh"
    assert isinstance(road_network_mesh.get_mesh(), Mesh3D), "get_mesh() does not return a Mesh3D"

def test_lanes_mesh(road_network_mesh):
    """Test the lanes mesh and its attributes."""
    lanes_mesh = road_network_mesh.lanes_mesh
    assert isinstance(lanes_mesh, LanesMesh), "lanes_mesh is not a LanesMesh"
    
    print(f"LanesMesh vertices exist: {hasattr(lanes_mesh, 'vertices')}")
    print(f"LanesMesh vertices length: {len(lanes_mesh.vertices) if hasattr(lanes_mesh, 'vertices') else 'N/A'}")
    if not hasattr(lanes_mesh, "vertices") or not lanes_mesh.vertices:
        pytest.skip("No vertices in lanes_mesh")
    
    vert_idx = 0
    assert isinstance(lanes_mesh.get_lanesec_s0(vert_idx), (float, int)), "get_lanesec_s0 does not return a number"
    assert isinstance(lanes_mesh.get_lane_id(vert_idx), (str, int)), "get_lane_id does not return a string or int"
    assert isinstance(lanes_mesh.get_idx_interval_lanesec(vert_idx), (list, tuple)), "get_idx_interval_lanesec does not return a list or tuple"
    assert isinstance(lanes_mesh.get_idx_interval_lane(vert_idx), (list, tuple)), "get_idx_interval_lane does not return a list or tuple"
    assert isinstance(lanes_mesh.get_lane_outline_indices(), (list, tuple)), "get_lane_outline_indices does not return a list or tuple"
    assert isinstance(lanes_mesh.lanesec_start_indices, dict), "lanesec_start_indices is not a dict"
    assert isinstance(lanes_mesh.lane_start_indices, dict), "lane_start_indices is not a dict"

def test_roadmarks_mesh(road_network_mesh):
    """Test the roadmarks mesh and its attributes."""
    roadmarks_mesh = road_network_mesh.roadmarks_mesh
    assert isinstance(roadmarks_mesh, RoadmarksMesh), "roadmarks_mesh is not a RoadmarksMesh"
    
    print(f"RoadmarksMesh vertices exist: {hasattr(roadmarks_mesh, 'vertices')}")
    print(f"RoadmarksMesh vertices length: {len(roadmarks_mesh.vertices) if hasattr(roadmarks_mesh, 'vertices') else 'N/A'}")
    if not hasattr(roadmarks_mesh, "vertices") or not roadmarks_mesh.vertices:
        pytest.skip("No vertices in roadmarks_mesh")
    
    vert_idx = 0
    assert isinstance(roadmarks_mesh.get_roadmark_type(vert_idx), str), "get_roadmark_type does not return a string"
    assert isinstance(roadmarks_mesh.get_idx_interval_roadmark(vert_idx), (list, tuple)), "get_idx_interval_roadmark does not return a list or tuple"
    assert isinstance(roadmarks_mesh.get_roadmark_outline_indices(), (list, tuple)), "get_roadmark_outline_indices does not return a list or tuple"
    assert isinstance(roadmarks_mesh.roadmark_type_start_indices, dict), "roadmark_type_start_indices is not a dict"

def test_road_objects_mesh(road_network_mesh):
    """Test the road objects mesh and its attributes."""
    road_objects_mesh = road_network_mesh.road_objects_mesh
    assert isinstance(road_objects_mesh, RoadObjectsMesh), "road_objects_mesh is not a RoadObjectsMesh"
    
    print(f"RoadObjectsMesh vertices exist: {hasattr(road_objects_mesh, 'vertices')}")
    print(f"RoadObjectsMesh vertices length: {len(road_objects_mesh.vertices) if hasattr(road_objects_mesh, 'vertices') else 'N/A'}")
    if not hasattr(road_objects_mesh, "vertices") or not road_objects_mesh.vertices:
        pytest.skip("No vertices in road_objects_mesh")
    
    vert_idx = 0
    assert isinstance(road_objects_mesh.get_road_object_id(vert_idx), (str, int)), "get_road_object_id does not return a string or int"
    assert isinstance(road_objects_mesh.get_idx_interval_road_object(vert_idx), (list, tuple)), "get_idx_interval_road_object does not return a list or tuple"
    assert isinstance(road_objects_mesh.road_object_start_indices, dict), "road_object_start_indices is not a dict"

def test_road_signals_mesh(road_network_mesh):
    """Test the road signals mesh and its attributes."""
    road_signals_mesh = road_network_mesh.road_signals_mesh
    assert isinstance(road_signals_mesh, RoadSignalsMesh), "road_signals_mesh is not a RoadSignalsMesh"
    
    print(f"RoadSignalsMesh vertices exist: {hasattr(road_signals_mesh, 'vertices')}")
    print(f"RoadSignalsMesh vertices length: {len(road_signals_mesh.vertices) if hasattr(road_signals_mesh, 'vertices') else 'N/A'}")
    if not hasattr(road_signals_mesh, "vertices") or not road_signals_mesh.vertices:
        pytest.skip("No vertices in road_signals_mesh")
    
    vert_idx = 0
    assert isinstance(road_signals_mesh.get_road_signal_id(vert_idx), (str, int)), "get_road_signal_id does not return a string or int"
    assert isinstance(road_signals_mesh.get_idx_interval_signal(vert_idx), (list, tuple)), "get_idx_interval_signal does not return a list or tuple"
    assert isinstance(road_signals_mesh.road_signal_start_indices, dict), "road_signal_start_indices is not a dict"

def test_routing_graph(odr_map):
    """Test the routing graph."""
    routing_graph = odr_map.get_routing_graph()
    assert isinstance(routing_graph, RoutingGraph), "routing_graph is not a RoutingGraph"
