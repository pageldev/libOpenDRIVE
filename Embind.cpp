#ifdef __EMSCRIPTEN__

    #include "LaneSection.h"
    #include "Lanes.h"
    #include "OpenDriveMap.h"
    #include "RefLine.h"
    #include "Road.h"
    #include "Utils.hpp"

    #include <emscripten/bind.h>

namespace odr
{
EMSCRIPTEN_BINDINGS(OpenDriveMap)
{
    /* arrays */
    emscripten::value_array<std::array<size_t, 2>>("array<size_t, 2>").element(emscripten::index<0>()).element(emscripten::index<1>());
    emscripten::value_array<Vec2D>("Vec2D").element(emscripten::index<0>()).element(emscripten::index<1>());
    emscripten::value_array<Vec3D>("Vec3D").element(emscripten::index<0>()).element(emscripten::index<1>()).element(emscripten::index<2>());

    /* vectors */
    emscripten::register_vector<size_t>("vector<size_t>");
    emscripten::register_vector<int>("vector<int>");
    emscripten::register_vector<double>("vector<double>");
    emscripten::register_vector<Vec2D>("vector<Vec2D>");
    emscripten::register_vector<Vec3D>("vector<Vec3D>");
    emscripten::register_vector<std::string>("vector<string>");
    emscripten::register_vector<std::vector<Vec3D>>("vector<vector<Vec3D>>");
    emscripten::register_vector<Mesh3D>("vector<Mesh3D>");
    emscripten::register_vector<RoadMark>("vector<RoadMark>");

    /* maps */
    emscripten::register_map<size_t, std::string>("map<size_t, string>");
    emscripten::register_map<size_t, double>("map<size_t, double>");
    emscripten::register_map<size_t, int>("map<size_t, int>");
    emscripten::register_map<int, double>("map<int, double>");
    emscripten::register_map<double, std::shared_ptr<RoadGeometry>>("map<double, shared_ptr<RoadGeometry>>");
    emscripten::register_map<std::string, std::shared_ptr<Road>>("map<string, shared_ptr<Road>>");
    emscripten::register_map<int, std::shared_ptr<Lane>>("map<int, shared_ptr<Lane>>");
    emscripten::register_map<double, std::shared_ptr<LaneSection>>("map<double, shared_ptr<LaneSection>>");

    /* classes */
    emscripten::class_<RoadGeometry>("RoadGeometry")
        .smart_ptr<std::shared_ptr<RoadGeometry>>("shared_ptr<RoadGeometry>")
        .function("get_xy", &RoadGeometry::get_xy)
        .function("get_grad", &RoadGeometry::get_grad)
        .property("s0", &RoadGeometry::s0);

    emscripten::class_<CubicSpline>("CubicSpline")
        .constructor<>()
        .smart_ptr<std::shared_ptr<CubicSpline>>("shared_ptr<CubicSpline>")
        .function("size", &CubicSpline::size)
        .function("get", &CubicSpline::get)
        .function("get_grad", &CubicSpline::get_grad)
        .function("get_max", &CubicSpline::get_max);

    emscripten::class_<RefLine>("RefLine")
        .constructor<double>()
        .smart_ptr<std::shared_ptr<RefLine>>("shared_ptr<RefLine>")
        .function("get_xyz", &RefLine::get_xyz)
        .function("get_grad", &RefLine::get_grad)
        .function("get_line", &RefLine::get_line)
        .function("match", &RefLine::match)
        .property("length", &RefLine::length)
        .property("elevation_profile", &RefLine::elevation_profile)
        .property("s0_to_geometry", &RefLine::s0_to_geometry);

    emscripten::class_<Mesh3D>("Mesh3D")
        .property("vertices", &Mesh3D::vertices)
        .property("indices", &Mesh3D::indices)
        .property("st_coordinates", &Mesh3D::st_coordinates);

    emscripten::class_<RoadMark>("RoadMark")
        .property("s_start", &RoadMark::s_start)
        .property("s_end", &RoadMark::s_end)
        .property("t_offset", &RoadMark::t_offset)
        .property("width", &RoadMark::width);

    emscripten::class_<Lane>("Lane")
        .constructor<int, bool, std::string>()
        .smart_ptr<std::shared_ptr<Lane>>("shared_ptr<Lane>")
        .function("get_surface_pt", &Lane::get_surface_pt)
        .function("get_mesh", &Lane::get_mesh)
        .function("get_roadmarks", &Lane::get_roadmarks)
        .function("get_roadmark_mesh", &Lane::get_roadmark_mesh)
        .property("inner_border", &Lane::inner_border)
        .property("outer_border", &Lane::outer_border)
        .property("id", &Lane::id)
        .property("type", &Lane::type);

    emscripten::class_<LaneSection>("LaneSection")
        .constructor<double>()
        .smart_ptr<std::shared_ptr<LaneSection>>("shared_ptr<LaneSection>")
        .function("get_end", &LaneSection::get_end)
        .function("get_lane", emscripten::select_overload<std::shared_ptr<Lane>(double, double)>(&LaneSection::get_lane))
        .property("s0", &LaneSection::s0)
        .property("id_to_lane", &LaneSection::id_to_lane);

    emscripten::class_<Road>("Road")
        .constructor<>()
        .smart_ptr<std::shared_ptr<Road>>("shared_ptr<Road>")
        .function("get_lanesection", emscripten::select_overload<std::shared_ptr<LaneSection>(double)>(&Road::get_lanesection))
        .function("get_xyz", &Road::get_xyz)
        .property("id", &Road::id)
        .property("junction", &Road::junction)
        .property("length", &Road::length)
        .property("lane_offset", &Road::lane_offset)
        .property("lane_offset", &Road::superelevation)
        .property("ref_line", &Road::ref_line)
        .property("s_to_lanesection", &Road::s_to_lanesection);

    emscripten::class_<MeshUnion, emscripten::base<Mesh3D>>("MeshUnion")
        .function("get_road_id", &MeshUnion::get_road_id)
        .function("get_lanesec_s0", &MeshUnion::get_lanesec_s0)
        .function("get_lane_id", &MeshUnion::get_lane_id)
        .function("get_idx_interval_road", &MeshUnion::get_idx_interval_road)
        .function("get_idx_interval_lanesec", &MeshUnion::get_idx_interval_lanesec)
        .function("get_idx_interval_lane", &MeshUnion::get_idx_interval_lane)
        .property("road_start_indices", &MeshUnion::road_start_indices)
        .property("lanesec_start_indices", &MeshUnion::lanesec_start_indices)
        .property("lane_start_indices", &MeshUnion::lane_start_indices);

    emscripten::class_<LaneMeshUnion, emscripten::base<MeshUnion>>("LaneMeshUnion")
        .function("get_lane_outline_indices", &LaneMeshUnion::get_lane_outline_indices);

    emscripten::class_<RoadmarkMeshUnion, emscripten::base<MeshUnion>>("RoadmarkMeshUnion")
        .function("get_roadmark_type", &RoadmarkMeshUnion::get_roadmark_type)
        .function("get_idx_interval_roadmark", &RoadmarkMeshUnion::get_idx_interval_roadmark)
        .function("get_roadmark_outline_indices", &RoadmarkMeshUnion::get_roadmark_outline_indices)        
        .property("roadmark_type_start_indices", &RoadmarkMeshUnion::roadmark_type_start_indices);

    emscripten::class_<RoadNetworkMesh>("RoadNetworkMesh")
        .property("lane_mesh_union", &RoadNetworkMesh::lane_mesh_union)
        .property("roadmark_mesh_union", &RoadNetworkMesh::roadmark_mesh_union);

    emscripten::class_<OpenDriveMap>("OpenDriveMap")
        .constructor<std::string, bool, bool>()
        .function("get_refline_segments", &OpenDriveMap::get_refline_lines)
        .function("get_mesh", &OpenDriveMap::get_mesh)
        .property("xodr_file", &OpenDriveMap::xodr_file)
        .property("roads", &OpenDriveMap::roads)
        .property("x_offs", &OpenDriveMap::x_offs)
        .property("y_offs", &OpenDriveMap::y_offs);
}

} // namespace odr

#endif