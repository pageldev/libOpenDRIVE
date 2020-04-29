#ifdef __EMSCRIPTEN__
#include "OpenDriveMap.h"
#include "Road.h"
#include "Lanes.h"
#include "Geometries/RoadGeometry.h"

#include <emscripten/bind.h>
#include <map>
#include <memory>
#include <string>

struct RoadGeometryWrapper : public emscripten::wrapper<RoadGeometry>
{
    EMSCRIPTEN_WRAPPER(RoadGeometryWrapper);

    virtual Point2D get_point(double s, double t) override
    {
        return call<Point2D>("get_point", s, t);
    }
};

EMSCRIPTEN_BINDINGS(OpenDriveMap)
{
    emscripten::register_vector<int>("vector<int>");
    emscripten::register_vector<double>("vector<double>");

    emscripten::value_object<Point2D>("Point2D")
        .field("x", &Point2D::x)
        .field("age", &Point2D::y);

    emscripten::enum_<Geometry_type>("Geometry_type")
        .value("Line", Geometry_type::Line)
        .value("Spiral", Geometry_type::Spiral)
        .value("Arc", Geometry_type::Arc)
        .value("ParamPoly3", Geometry_type::ParamPoly3);

    emscripten::class_<RoadGeometry>("RoadGeometry")
        .smart_ptr<std::shared_ptr<RoadGeometry>>("shared_ptr<RoadGeometry>")
        .allow_subclass<RoadGeometryWrapper>("RoadGeometryWrapper", emscripten::constructor<double, double, double, double, double, Geometry_type>())
        .function("get_point", &RoadGeometry::get_point, emscripten::pure_virtual())
        .property("type", &RoadGeometry::type)
        .property("s0", &RoadGeometry::s0)
        .property("x0", &RoadGeometry::x0)
        .property("y0", &RoadGeometry::y0)
        .property("hdg0", &RoadGeometry::hdg0)
        .property("length", &RoadGeometry::length);

    emscripten::class_<Arc, emscripten::base<RoadGeometry>>("Arc")
        .property("curvature", &Arc::curvature);

    emscripten::class_<Line, emscripten::base<RoadGeometry>>("Line");

    emscripten::class_<ParamPoly3, emscripten::base<RoadGeometry>>("ParamPoly3")
        .property("aU", &ParamPoly3::aU)
        .property("bU", &ParamPoly3::bU)
        .property("cU", &ParamPoly3::cU)
        .property("dU", &ParamPoly3::dU)
        .property("aV", &ParamPoly3::aV)
        .property("bV", &ParamPoly3::bV)
        .property("cV", &ParamPoly3::cV)
        .property("dV", &ParamPoly3::dV);

    emscripten::class_<Spiral, emscripten::base<RoadGeometry>>("Spiral")
        .property("curv_start", &Spiral::curv_start)
        .property("curv_end", &Spiral::curv_end)
        .property("c_dot", &Spiral::c_dot);

    emscripten::register_map<int, std::shared_ptr<Lane>>("map<int, shared_ptr<Lane>>");

    emscripten::class_<LaneSection>("LaneSection")
        .smart_ptr<std::shared_ptr<LaneSection>>("shared_ptr<LaneSection>")
        .constructor<double>()
        .property("s0", &LaneSection::s0)
        .property("road", &LaneSection::road)
        .property("lanes", &LaneSection::lanes);

    emscripten::register_map<double, std::shared_ptr<ElevationProfile>>("map<double, shared_ptr<ElevationProfile>>");
    emscripten::register_map<double, std::shared_ptr<RoadGeometry>>("map<double, shared_ptr<RoadGeometry>>");
    emscripten::register_map<double, std::shared_ptr<LaneSection>>("map<double, shared_ptr<LaneSection>>");
    emscripten::register_map<double, std::shared_ptr<LaneOffset>>("map<double, shared_ptr<LaneOffset>>");

    emscripten::class_<Road>("Road")
        .smart_ptr<std::shared_ptr<Road>>("shared_ptr<Road>")
        .constructor<double, int, int, std::map<double, std::shared_ptr<RoadGeometry>>>()
        .function("get_refline_point", &Road::get_refline_point)
        .property("id", &Road::id)
        .property("length", &Road::length)
        .property("junction", &Road::junction)
        .property("elevation_profiles", &Road::elevation_profiles)
        .property("geometries", &Road::geometries)
        .property("lane_sections", &Road::lane_sections)
        .property("lane_offsets", &Road::lane_offsets);

    emscripten::register_map<int, std::shared_ptr<Road>>("map<int, shared_ptr<Road>>");

    emscripten::class_<OpenDriveMap>("OpenDriveMap")
        .constructor<std::string>()
        .function("dump_json", &OpenDriveMap::dump_json)
        .property("xodr_file", &OpenDriveMap::xodr_file)
        .property("roads", &OpenDriveMap::roads);
}
#endif