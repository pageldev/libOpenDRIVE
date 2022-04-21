#ifdef __EMSCRIPTEN__

    #include "Lane.h"
    #include "LaneSection.h"
    #include "Mesh.h"
    #include "OpenDriveMap.h"
    #include "RefLine.h"
    #include "Road.h"
    #include "RoadNetworkMesh.h"
    #include "Utils.hpp"
    #include "ViewerUtils.h"

    #include <cstddef>
    #include <emscripten/bind.h>

namespace odr
{
EMSCRIPTEN_BINDINGS(OpenDriveMap)
{
    /* arrays */
    emscripten::value_array<std::array<std::size_t, 2>>("array<std::size_t, 2>").element(emscripten::index<0>()).element(emscripten::index<1>());
    emscripten::value_array<Vec2D>("Vec2D").element(emscripten::index<0>()).element(emscripten::index<1>());
    emscripten::value_array<Vec3D>("Vec3D").element(emscripten::index<0>()).element(emscripten::index<1>()).element(emscripten::index<2>());

    /* vectors */
    emscripten::register_vector<std::size_t>("vector<std::size_t>");
    emscripten::register_vector<std::uint32_t>("vector<std::uint32_t>");
    emscripten::register_vector<int>("vector<int>");
    emscripten::register_vector<double>("vector<double>");
    emscripten::register_vector<Vec2D>("vector<Vec2D>");
    emscripten::register_vector<Vec3D>("vector<Vec3D>");
    emscripten::register_vector<std::string>("vector<string>");
    emscripten::register_vector<std::vector<Vec3D>>("vector<vector<Vec3D>>");
    emscripten::register_vector<Mesh3D>("vector<Mesh3D>");
    emscripten::register_vector<RoadMark>("vector<RoadMark>");

    /* maps */
    emscripten::register_map<std::size_t, std::string>("map<std::size_t, string>");
    emscripten::register_map<std::size_t, double>("map<std::size_t, double>");
    emscripten::register_map<std::size_t, int>("map<std::size_t, int>");
    emscripten::register_map<int, double>("map<int, double>");
    emscripten::register_map<double, std::shared_ptr<RoadGeometry>>("map<double, shared_ptr<RoadGeometry>>");
    emscripten::register_map<std::string, std::shared_ptr<Road>>("map<string, shared_ptr<Road>>");
    emscripten::register_map<int, std::shared_ptr<Lane>>("map<int, shared_ptr<Lane>>");
    emscripten::register_map<double, std::shared_ptr<LaneSection>>("map<double, shared_ptr<LaneSection>>");

    /* classes */
    emscripten::class_<Mesh3D>("Mesh3D")
        .function("get_obj", &Mesh3D::get_obj)
        .property("vertices", &Mesh3D::vertices)
        .property("indices", &Mesh3D::indices)
        .property("normals", &Mesh3D::normals)
        .property("st_coordinates", &Mesh3D::st_coordinates);

    emscripten::class_<RoadsMesh, emscripten::base<Mesh3D>>("RoadsMesh")
        .function("get_road_id", &RoadsMesh::get_road_id)
        .function("get_idx_interval_road", &RoadsMesh::get_idx_interval_road)
        .property("road_start_indices", &RoadsMesh::road_start_indices);

    emscripten::class_<LanesMesh, emscripten::base<RoadsMesh>>("LanesMesh")
        .function("get_lanesec_s0", &LanesMesh::get_lanesec_s0)
        .function("get_lane_id", &LanesMesh::get_lane_id)
        .function("get_idx_interval_lanesec", &LanesMesh::get_idx_interval_lanesec)
        .function("get_idx_interval_lane", &LanesMesh::get_idx_interval_lane)
        .function("get_lane_outline_indices", &LanesMesh::get_lane_outline_indices)
        .property("lanesec_start_indices", &LanesMesh::lanesec_start_indices)
        .property("lane_start_indices", &LanesMesh::lane_start_indices);

    emscripten::class_<RoadmarksMesh, emscripten::base<LanesMesh>>("RoadmarksMesh")
        .function("get_roadmark_type", &RoadmarksMesh::get_roadmark_type)
        .function("get_idx_interval_roadmark", &RoadmarksMesh::get_idx_interval_roadmark)
        .function("get_roadmark_outline_indices", &RoadmarksMesh::get_roadmark_outline_indices)
        .property("roadmark_type_start_indices", &RoadmarksMesh::roadmark_type_start_indices);

    emscripten::class_<RoadObjectsMesh, emscripten::base<RoadsMesh>>("RoadObjectsMesh")
        .function("get_road_object_id", &RoadObjectsMesh::get_road_object_id)
        .function("get_idx_interval_road_object", &RoadObjectsMesh::get_idx_interval_road_object)
        .property("road_object_start_indices", &RoadObjectsMesh::road_object_start_indices);

    emscripten::class_<RoadNetworkMesh>("RoadNetworkMesh")
        .property("lanes_mesh", &RoadNetworkMesh::lanes_mesh)
        .property("roadmarks_mesh", &RoadNetworkMesh::roadmarks_mesh)
        .property("road_objects_mesh", &RoadNetworkMesh::road_objects_mesh);

    emscripten::value_object<OpenDriveMapConfig>("OpenDriveMapConfig")
        .field("with_lateralProfile", &OpenDriveMapConfig::with_lateralProfile)
        .field("with_laneHeight", &OpenDriveMapConfig::with_laneHeight)
        .field("with_road_objects", &OpenDriveMapConfig::with_road_objects)
        .field("center_map", &OpenDriveMapConfig::center_map)
        .field("abs_z_for_for_local_road_obj_outline", &OpenDriveMapConfig::abs_z_for_for_local_road_obj_outline);

    emscripten::class_<OpenDriveMap>("OpenDriveMap")
        .constructor<std::string, OpenDriveMapConfig>()
        .property("xodr_file", &OpenDriveMap::xodr_file)
        .property("x_offs", &OpenDriveMap::x_offs)
        .property("y_offs", &OpenDriveMap::y_offs);

    emscripten::function("get_road_network_mesh", &get_road_network_mesh);
    emscripten::function("get_refline_segments", &get_refline_segments);
}

} // namespace odr

#endif
