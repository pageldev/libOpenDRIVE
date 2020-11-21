#ifdef __EMSCRIPTEN__
    #include "Geometries/Geometries.h"
    #include "Geometries/RoadGeometry.h"
    #include "Lanes.h"
    #include "OpenDriveMap.h"
    #include "RefLine.h"
    #include "Road.h"

    #include <emscripten/bind.h>
    #include <map>
    #include <memory>
    #include <string>

namespace odr
{
struct RoadGeometryWrapper : public emscripten::wrapper<RoadGeometry>
{
    EMSCRIPTEN_WRAPPER(RoadGeometryWrapper);

    virtual void update() override { return call<void>("update"); }

    virtual Vec2D get_xy(double s) const override { return call<Vec2D>("get_xy", s); }

    virtual Vec2D get_grad(double s) const override { return call<Vec2D>("get_grad", s); }
};

EMSCRIPTEN_BINDINGS(OpenDriveMap)
{
    emscripten::register_vector<int>("vector<int>");
    emscripten::register_vector<double>("vector<double>");

    emscripten::value_array<Vec2D>("Vec2d").element(emscripten::index<0>()).element(emscripten::index<1>());

    emscripten::value_array<Vec3D>("Vec3d").element(emscripten::index<0>()).element(emscripten::index<1>()).element(emscripten::index<2>());

    emscripten::register_vector<Vec3D>("vector<Vec3D>");

    emscripten::class_<Box2D>("Box2D")
        .function("get_distance", &Box2D::get_distance)
        .property("min", &Box2D::min)
        .property("max", &Box2D::max)
        .property("width", &Box2D::width)
        .property("height", &Box2D::height)
        .property("center", &Box2D::center);

    emscripten::class_<Poly3>("Poly3")
        .smart_ptr<std::shared_ptr<Poly3>>("shared_ptr<Poly3>")
        .constructor<double, double, double, double, double>()
        .function("get", &Poly3::get)
        .function("get_grad", &Poly3::get_grad)
        .property("s0", &Poly3::s0)
        .property("a", &Poly3::a)
        .property("b", &Poly3::b)
        .property("c", &Poly3::c)
        .property("d", &Poly3::d);

    emscripten::register_map<double, std::shared_ptr<Poly3>>("map<double, shared_ptr<Poly3>>");

    emscripten::class_<CubicSpline>("CubicSpline")
        .constructor<>()
        .function("size", &CubicSpline::size)
        .function("get", &CubicSpline::get)
        .function("get_grad", &CubicSpline::get_grad)
        .function("get_poly", &CubicSpline::get_poly)
        .property("s0_to_poly", &CubicSpline::s0_to_poly);

    emscripten::enum_<GeometryType>("GeometryType")
        .value("Line", GeometryType::Line)
        .value("Spiral", GeometryType::Spiral)
        .value("Arc", GeometryType::Arc)
        .value("ParamPoly3", GeometryType::ParamPoly3);

    emscripten::class_<RoadGeometry>("RoadGeometry")
        .smart_ptr<std::shared_ptr<RoadGeometry>>("shared_ptr<RoadGeometry>")
        .smart_ptr<std::shared_ptr<const RoadGeometry>>("shared_ptr<const RoadGeometry>")
        .allow_subclass<RoadGeometryWrapper>("RoadGeometryWrapper",
                                             emscripten::constructor<double, double, double, double, double, GeometryType, std::shared_ptr<Road>>())
        .function("update", &RoadGeometry::update, emscripten::pure_virtual())
        .function("get_xy", &RoadGeometry::get_xy, emscripten::pure_virtual())
        .function("get_grad", &RoadGeometry::get_grad, emscripten::pure_virtual())
        .property("type", &RoadGeometry::type)
        .property("s0", &RoadGeometry::s0)
        .property("x0", &RoadGeometry::x0)
        .property("y0", &RoadGeometry::y0)
        .property("hdg0", &RoadGeometry::hdg0)
        .property("length", &RoadGeometry::length)
        .property("bounding_box", &RoadGeometry::bounding_box)
        .property("road", &RoadGeometry::road);

    emscripten::class_<Arc, emscripten::base<RoadGeometry>>("Arc")
        .constructor<double, double, double, double, double, double, std::shared_ptr<Road>>()
        .property("curvature", &Arc::curvature);

    emscripten::class_<Line, emscripten::base<RoadGeometry>>("Line").constructor<double, double, double, double, double, std::shared_ptr<Road>>();

    emscripten::class_<ParamPoly3, emscripten::base<RoadGeometry>>("ParamPoly3")
        .constructor<double, double, double, double, double, double, double, double, double, double, double, double, double, std::shared_ptr<Road>>()
        .property("aU", &ParamPoly3::aU)
        .property("bU", &ParamPoly3::bU)
        .property("cU", &ParamPoly3::cU)
        .property("dU", &ParamPoly3::dU)
        .property("aV", &ParamPoly3::aV)
        .property("bV", &ParamPoly3::bV)
        .property("cV", &ParamPoly3::cV)
        .property("dV", &ParamPoly3::dV);

    emscripten::class_<Spiral, emscripten::base<RoadGeometry>>("Spiral")
        .constructor<double, double, double, double, double, double, double, std::shared_ptr<Road>>()
        .property("curv_start", &Spiral::curv_start)
        .property("curv_end", &Spiral::curv_end)
        .property("c_dot", &Spiral::c_dot);

    emscripten::class_<Lane>("Lane")
        .smart_ptr<std::shared_ptr<Lane>>("shared_ptr<Lane>")
        .constructor<int, std::string>()
        .function("get_outer_border_pt", &Lane::get_outer_border_pt)
        .property("id", &Lane::id)
        .property("type", &Lane::type)
        .property("lane_section", &Lane::lane_section)
        .property("lane_width", &Lane::lane_width);

    emscripten::register_map<int, std::shared_ptr<Lane>>("map<int, shared_ptr<Lane>>");
    emscripten::register_map<int, double>("map<int, double>");

    emscripten::class_<LaneSection>("LaneSection")
        .smart_ptr<std::shared_ptr<LaneSection>>("shared_ptr<LaneSection>")
        .constructor<double>()
        .function("get_lanes", &LaneSection::get_lanes)
        .function("get_lane", &LaneSection::get_lane)
        .function("get_lane_borders", &LaneSection::get_lane_borders)
        .property("s0", &LaneSection::s0)
        .property("road", &LaneSection::road)
        .property("id_to_lane", &LaneSection::id_to_lane);

    emscripten::register_map<double, std::shared_ptr<RoadGeometry>>("map<double, shared_ptr<RoadGeometry>>");
    emscripten::register_map<double, std::shared_ptr<LaneSection>>("map<double, shared_ptr<LaneSection>>");

    emscripten::class_<RefLine>("RefLine")
        .smart_ptr<std::shared_ptr<RefLine>>("shared_ptr<RefLine>")
        .constructor<double>()
        .function("get_geometries", &RefLine::get_geometries)
        .function("get_geometry", &RefLine::get_geometry)
        .function("get_xyz", &RefLine::get_xyz)
        .function("get_grad", &RefLine::get_grad)
        .function("match", &RefLine::match)
        .property("length", &RefLine::length)
        .property("elevation_profile", &RefLine::elevation_profile)
        .property("s0_to_geometry", &RefLine::s0_to_geometry);

    emscripten::class_<Road>("Road")
        .smart_ptr<std::shared_ptr<Road>>("shared_ptr<Road>")
        .constructor<double, int, int>()
        .function("get_lane", &Road::get_lane)
        .function("get_lanesection", &Road::get_lanesection)
        .function("get_lanesections", &Road::get_lanesections)
        .function("get_xyz", &Road::get_xyz)
        .property("id", &Road::id)
        .property("junction", &Road::junction)
        .property("length", &Road::length)
        .property("lane_offset", &Road::lane_offset)
        .property("lane_offset", &Road::superelevation)
        .property("ref_line", &Road::ref_line)
        .property("s0_to_lanesection", &Road::s0_to_lanesection);

    emscripten::register_map<int, std::shared_ptr<Road>>("map<int, shared_ptr<Road>>");

    emscripten::class_<OpenDriveMap>("OpenDriveMap")
        .constructor<std::string>()
        .function("get_roads", &OpenDriveMap::get_roads)
        .property("xodr_file", &OpenDriveMap::xodr_file)
        .property("roads", &OpenDriveMap::roads);
}

} // namespace odr
#endif