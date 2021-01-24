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
    emscripten::value_array<Vec2D>("Vec2D").element(emscripten::index<0>()).element(emscripten::index<1>());
    emscripten::value_array<Vec3D>("Vec3D").element(emscripten::index<0>()).element(emscripten::index<1>()).element(emscripten::index<2>());

    /* containers */
    emscripten::register_vector<int>("vector<int>");
    emscripten::register_vector<double>("vector<double>");
    emscripten::register_map<int, double>("map<int, double>");
    emscripten::register_vector<Vec3D>("vector<Vec3D>");
    emscripten::register_vector<size_t>("vector<size_t>");
    emscripten::register_vector<std::string>("vector<string>");
    emscripten::register_vector<std::vector<Vec3D>>("vector<vector<Vec3D>>");
    emscripten::register_vector<LaneVertices>("vector<LaneVertices>");
    emscripten::register_vector<Mesh3D>("vector<Mesh3D>");
    emscripten::register_vector<RoadMarkLines>("vector<RoadMarkLines>");
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
        .function("get_grad", &CubicSpline::get_grad);

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
        .property("indices", &Mesh3D::indices);

    emscripten::class_<RoadMarkLines>("RoadMarkLines")
        .function("generate_meshes", &RoadMarkLines::generate_meshes)
        .property("lane_id", &RoadMarkLines::lane_id)
        .property("width", &RoadMarkLines::width)
        .property("length", &RoadMarkLines::length)
        .property("space", &RoadMarkLines::space)
        .property("height", &RoadMarkLines::height)
        .property("name", &RoadMarkLines::name)
        .property("rule", &RoadMarkLines::rule)
        .property("type", &RoadMarkLines::type)
        .property("weight", &RoadMarkLines::weight)
        .property("color", &RoadMarkLines::color)
        .property("material", &RoadMarkLines::material)
        .property("laneChange", &RoadMarkLines::laneChange)
        .property("lines", &RoadMarkLines::lines);

    emscripten::class_<Lane>("Lane")
        .constructor<int, bool, std::string>()
        .smart_ptr<std::shared_ptr<Lane>>("shared_ptr<Lane>")
        .function("get_surface_pt", &Lane::get_surface_pt)
        .function("get_mesh", &Lane::get_mesh)
        .property("id", &Lane::id)
        .property("type", &Lane::type);

    emscripten::class_<LaneSection>("LaneSection")
        .constructor<double>()
        .smart_ptr<std::shared_ptr<LaneSection>>("shared_ptr<LaneSection>")
        .function("get_end", &LaneSection::get_end)
        .function("get_lane", emscripten::select_overload<std::shared_ptr<Lane>(double, double)>(&LaneSection::get_lane))
        .function("get_roadmark_lines", emscripten::select_overload<std::vector<RoadMarkLines>(int, double) const>(&LaneSection::get_roadmark_lines))
        .property("s0", &LaneSection::s0)
        .property("id_to_lane", &LaneSection::id_to_lane);

    emscripten::class_<LaneVertices>("LaneVertices")
        .constructor<>()
        .property("vertices", &LaneVertices::vertices)
        .property("indices", &LaneVertices::indices)
        .property("lane_id", &LaneVertices::lane_id)
        .property("type", &LaneVertices::type);

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

    emscripten::class_<OpenDriveMap>("OpenDriveMap")
        .constructor<std::string, bool, bool>()
        .property("xodr_file", &OpenDriveMap::xodr_file)
        .property("roads", &OpenDriveMap::roads);
}

} // namespace odr

#endif