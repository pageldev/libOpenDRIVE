#include "OpenDriveMap.h"
#include "Geometries/Arc.h"
#include "Geometries/CubicSpline.h"
#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"
#include "Geometries/Spiral.h"
#include "LaneSection.h"
#include "Lanes.h"
#include "RefLine.h"
#include "Road.h"

#include <iostream>
#include <math.h>
#include <string>
#include <utility>

namespace odr
{
OpenDriveMap::OpenDriveMap(std::string xodr_file, bool with_lateralProfile, bool with_laneHeight, bool center_map, bool with_objects) :
    xodr_file(xodr_file)
{
    pugi::xml_parse_result result = this->xml_doc.load_file(xodr_file.c_str());
    if (!result)
        printf("%s\n", result.description());

    pugi::xml_node odr_node = this->xml_doc.child("OpenDRIVE");

    if (auto geoReference_node = odr_node.child("header").child("geoReference"))
        this->proj4 = geoReference_node.text().as_string();

    size_t cnt = 1;
    if (center_map)
    {
        for (pugi::xml_node road_node : odr_node.children("road"))
        {
            for (pugi::xml_node geometry_hdr_node : road_node.child("planView").children("geometry"))
            {
                const double x0 = geometry_hdr_node.attribute("x").as_double();
                this->x_offs = this->x_offs + ((x0 - this->x_offs) / cnt);
                const double y0 = geometry_hdr_node.attribute("y").as_double();
                this->y_offs = this->y_offs + ((y0 - this->y_offs) / cnt);
                cnt++;
            }
        }
    }

    for (pugi::xml_node road_node : odr_node.children("road"))
    {
        /* make road */
        std::shared_ptr<Road> road = std::make_shared<Road>();
        road->length = road_node.attribute("length").as_double();
        road->id = road_node.attribute("id").as_string();
        road->junction = road_node.attribute("junction").as_string();
        road->name = road_node.attribute("name").as_string();
        road->xml_node = road_node;
        this->roads[road->id] = road;

        CHECK_AND_REPAIR(road->length >= 0, "road::length < 0", road->length = 0);

        /* parse road links */
        for (bool is_predecessor : {true, false})
        {
            pugi::xml_node road_link_node =
                is_predecessor ? road_node.child("link").child("predecessor") : road_node.child("link").child("successor");
            if (road_link_node)
            {
                RoadLink& link = is_predecessor ? road->predecessor : road->successor;
                link.elementId = road_link_node.child("elementId").text().as_string();
                link.elementType = road_link_node.child("elementType").text().as_string();
                link.contactPoint = road_link_node.child("contactPoint").text().as_string();
                link.xml_node = road_link_node;
            }
        }

        /* parse road neighbors */
        for (pugi::xml_node road_neighbor_node : road_node.child("link").children("neighbor"))
        {
            RoadNeighbor road_neighbor;
            road_neighbor.elementId = road_neighbor_node.attribute("elementId").as_string();
            road_neighbor.side = road_neighbor_node.attribute("side").as_string();
            road_neighbor.direction = road_neighbor_node.attribute("direction").as_string();
            road_neighbor.xml_node = road_neighbor_node;
            road->neighbors.push_back(road_neighbor);
        }

        /* parse road type and speed */
        for (pugi::xml_node road_type_node : road_node.children("type"))
        {
            double      s = road_type_node.attribute("s").as_double();
            std::string type = road_type_node.attribute("type").as_string();

            CHECK_AND_REPAIR(s >= 0, "road::type::s < 0", s = 0);

            road->s_to_type[s] = type;
            if (pugi::xml_node node = road_type_node.child("speed"))
            {
                SpeedRecord speed_record;
                speed_record.max = node.attribute("max").as_string();
                speed_record.unit = node.attribute("unit").as_string();
                speed_record.xml_node = node;
                road->s_to_speed[s] = speed_record;
            }
        }

        /* make ref_line - parse road geometries */
        road->ref_line = std::make_shared<RefLine>(road->length);
        for (pugi::xml_node geometry_hdr_node : road_node.child("planView").children("geometry"))
        {
            double s0 = geometry_hdr_node.attribute("s").as_double();
            double x0 = geometry_hdr_node.attribute("x").as_double() - this->x_offs;
            double y0 = geometry_hdr_node.attribute("y").as_double() - this->y_offs;
            double hdg0 = geometry_hdr_node.attribute("hdg").as_double();
            double length = geometry_hdr_node.attribute("length").as_double();

            CHECK_AND_REPAIR(s0 >= 0, "road::planView::geometry::s < 0", s0 = 0);
            CHECK_AND_REPAIR(length >= 0, "road::planView::geometry::length < 0", length = 0);

            pugi::xml_node geometry_node = geometry_hdr_node.first_child();
            std::string    geometry_type = geometry_node.name();
            if (geometry_type == "line")
            {
                road->ref_line->s0_to_geometry[s0] = std::make_shared<Line>(s0, x0, y0, hdg0, length);
            }
            else if (geometry_type == "spiral")
            {
                double curv_start = geometry_node.attribute("curvStart").as_double();
                double curv_end = geometry_node.attribute("curvEnd").as_double();
                road->ref_line->s0_to_geometry[s0] = std::make_shared<Spiral>(s0, x0, y0, hdg0, length, curv_start, curv_end);
            }
            else if (geometry_type == "arc")
            {
                double curvature = geometry_node.attribute("curvature").as_double();
                road->ref_line->s0_to_geometry[s0] = std::make_shared<Arc>(s0, x0, y0, hdg0, length, curvature);
            }
            else if (geometry_type == "paramPoly3")
            {
                double aU = geometry_node.attribute("aU").as_double();
                double bU = geometry_node.attribute("bU").as_double();
                double cU = geometry_node.attribute("cU").as_double();
                double dU = geometry_node.attribute("dU").as_double();
                double aV = geometry_node.attribute("aV").as_double();
                double bV = geometry_node.attribute("bV").as_double();
                double cV = geometry_node.attribute("cV").as_double();
                double dV = geometry_node.attribute("dV").as_double();

                bool pRange_normalized = true;
                if (geometry_node.attribute("pRange") || geometry_hdr_node.attribute("pRange"))
                {
                    std::string pRange_str = geometry_node.attribute("pRange") ? geometry_node.attribute("pRange").as_string()
                                                                               : geometry_hdr_node.attribute("pRange").as_string();
                    std::transform(pRange_str.begin(), pRange_str.end(), pRange_str.begin(), [](unsigned char c) { return std::tolower(c); });
                    if (pRange_str == "arclength")
                        pRange_normalized = false;
                }
                road->ref_line->s0_to_geometry[s0] =
                    std::make_shared<ParamPoly3>(s0, x0, y0, hdg0, length, aU, bU, cU, dU, aV, bV, cV, dV, pRange_normalized);
            }
            else
            {
                printf("Could not parse %s\n", geometry_type.c_str());
                continue;
            }

            road->ref_line->s0_to_geometry.at(s0)->xml_node = geometry_node;
        }

        std::map<std::string /*x path query*/, CubicSpline&> cubic_spline_fields{
            {".//elevationProfile//elevation", road->ref_line->elevation_profile}, {".//lanes//laneOffset", road->lane_offset}};

        if (with_lateralProfile)
            cubic_spline_fields.insert({".//lateralProfile//superelevation", road->superelevation});

        /* parse elevation profiles, lane offsets, superelevation */
        for (auto entry : cubic_spline_fields)
        {
            pugi::xpath_node_set nodes = road_node.select_nodes(entry.first.c_str());
            for (pugi::xpath_node node : nodes)
            {
                double s0 = node.node().attribute("s").as_double();
                double a = node.node().attribute("a").as_double();
                double b = node.node().attribute("b").as_double();
                double c = node.node().attribute("c").as_double();
                double d = node.node().attribute("d").as_double();

                CHECK_AND_REPAIR(s0 >= 0, (entry.first + "::s < 0").c_str(), s0 = 0);

                entry.second.s0_to_poly[s0] = Poly3(s0, a, b, c, d);
            }
        }

        /* parse crossfall - has extra attribute side */
        if (with_lateralProfile)
        {
            for (pugi::xml_node crossfall_node : road_node.child("lateralProfile").children("crossfall"))
            {
                double s0 = crossfall_node.attribute("s").as_double();
                double a = crossfall_node.attribute("a").as_double();
                double b = crossfall_node.attribute("b").as_double();
                double c = crossfall_node.attribute("c").as_double();
                double d = crossfall_node.attribute("d").as_double();

                CHECK_AND_REPAIR(s0 >= 0, "road::lateralProfile::crossfall::s < 0", s0 = 0);

                Poly3 crossfall_poly(s0, a, b, c, d);
                road->crossfall.s0_to_poly[s0] = crossfall_poly;
                if (pugi::xml_attribute side = crossfall_node.attribute("side"))
                {
                    std::string side_str = side.as_string();
                    std::transform(side_str.begin(), side_str.end(), side_str.begin(), [](unsigned char c) { return std::tolower(c); });
                    if (side_str == "left")
                        road->crossfall.sides[s0] = Crossfall::Side::Left;
                    else if (side_str == "right")
                        road->crossfall.sides[s0] = Crossfall::Side::Right;
                    else
                        road->crossfall.sides[s0] = Crossfall::Side::Both;
                }
            }

            /* check for lateralProfile shape - not implemented yet */
            for (auto road_shape_node : road_node.child("lateralProfile").children("shape"))
                printf("Lateral Profile Shape not supported\n");
        }

        /* parse road lane sections and lanes */
        for (pugi::xml_node lane_section_node : road_node.child("lanes").children("laneSection"))
        {
            double s0 = lane_section_node.attribute("s").as_double();

            std::shared_ptr<LaneSection> lane_section = std::make_shared<LaneSection>(s0);
            lane_section->road = road;
            lane_section->xml_node = lane_section_node;
            road->s_to_lanesection[lane_section->s0] = lane_section;

            for (pugi::xpath_node lane_xpath_node : lane_section_node.select_nodes(".//lane"))
            {
                pugi::xml_node lane_node = lane_xpath_node.node();
                int            lane_id = lane_node.attribute("id").as_int();
                std::string    lane_type = lane_node.attribute("type").as_string();
                bool           level = lane_node.attribute("level").as_bool();

                std::shared_ptr<Lane> lane = std::make_shared<Lane>(lane_id, level, lane_type);
                lane->road = road;
                lane->lane_section = lane_section;
                lane->xml_node = lane_node;
                lane_section->id_to_lane[lane->id] = lane;

                for (pugi::xml_node lane_width_node : lane_node.children("width"))
                {
                    double s_offset = lane_width_node.attribute("sOffset").as_double();
                    double a = lane_width_node.attribute("a").as_double();
                    double b = lane_width_node.attribute("b").as_double();
                    double c = lane_width_node.attribute("c").as_double();
                    double d = lane_width_node.attribute("d").as_double();

                    CHECK_AND_REPAIR(s_offset >= 0, "lane::width::sOffset < 0", s_offset = 0);
                    lane->lane_width.s0_to_poly[s0 + s_offset] = Poly3(s0 + s_offset, a, b, c, d);
                }

                if (with_laneHeight)
                {
                    for (pugi::xml_node lane_height_node : lane_node.children("height"))
                    {
                        double s_offset = lane_height_node.attribute("sOffset").as_double();
                        double inner = lane_height_node.attribute("inner").as_double();
                        double outer = lane_height_node.attribute("outer").as_double();

                        CHECK_AND_REPAIR(s_offset >= 0, "lane::height::sOffset < 0", s_offset = 0);
                        lane->s_to_height_offset[s0 + s_offset] = HeightOffset{inner, outer};
                    }
                }

                for (pugi::xml_node roadmark_node : lane_node.children("roadMark"))
                {
                    RoadMarkGroup roadmark_group;
                    roadmark_group.xml_node = roadmark_node;
                    roadmark_group.s_offset = roadmark_node.attribute("sOffset").as_double(0);
                    roadmark_group.width = roadmark_node.attribute("width").as_double(-1);
                    roadmark_group.height = roadmark_node.attribute("height").as_double(0);
                    roadmark_group.type = roadmark_node.attribute("type").as_string("none");
                    roadmark_group.weight = roadmark_node.attribute("weight").as_string("standard");
                    roadmark_group.color = roadmark_node.attribute("color").as_string("standard");
                    roadmark_group.material = roadmark_node.attribute("material").as_string("standard");
                    roadmark_group.laneChange = roadmark_node.attribute("laneChange").as_string("both");

                    CHECK_AND_REPAIR(roadmark_group.s_offset >= 0, "lane::roadMark::sOffset < 0", roadmark_group.s_offset = 0);

                    if (pugi::xml_node roadmark_type_node = roadmark_node.child("type"))
                    {
                        std::string name = roadmark_type_node.attribute("name").as_string("");
                        double      line_width_1 = roadmark_type_node.attribute("width").as_double(-1);

                        for (pugi::xml_node roadmarks_line_node : roadmark_type_node.children("line"))
                        {
                            RoadMarksLine roadmarks_line;
                            roadmarks_line.name = name;
                            roadmarks_line.length = roadmarks_line_node.attribute("length").as_double(0);
                            roadmarks_line.space = roadmarks_line_node.attribute("space").as_double(0);
                            roadmarks_line.t_offset = roadmarks_line_node.attribute("tOffset").as_double(0);
                            roadmarks_line.s_offset = roadmarks_line_node.attribute("sOffset").as_double(0);
                            double line_width_0 = roadmarks_line_node.attribute("width").as_double(-1);
                            roadmarks_line.width = line_width_0 < 0 ? line_width_1 : line_width_0;
                            roadmarks_line.rule = roadmarks_line_node.attribute("rule").as_string("none");
                            roadmarks_line.xml_node = roadmarks_line_node;

                            CHECK_AND_REPAIR(roadmarks_line.length >= 0, "roadMark::type::line::length < 0", roadmarks_line.length = 0);
                            CHECK_AND_REPAIR(roadmarks_line.space >= 0, "roadMark::type::line::space < 0", roadmarks_line.space = 0);
                            CHECK_AND_REPAIR(roadmarks_line.s_offset >= 0, "roadMark::type::line::sOffset < 0", roadmarks_line.s_offset = 0);

                            roadmark_group.s_to_roadmarks_line[s0 + roadmark_group.s_offset + roadmarks_line.s_offset] = roadmarks_line;
                        }
                    }
                    lane->s_to_roadmark_group[s0 + roadmark_group.s_offset] = roadmark_group;
                }

                if (pugi::xml_node node = lane_node.child("link").child("predecessor"))
                    lane->predecessor = node.attribute("id").as_int();
                if (pugi::xml_node node = lane_node.child("link").child("successor"))
                    lane->successor = node.attribute("id").as_int();
            }

            /* derive lane borders from lane widths */
            auto id_lane_iter0 = lane_section->id_to_lane.find(0);
            if (id_lane_iter0 == lane_section->id_to_lane.end())
                throw std::runtime_error("lane section does not have lane #0");

            /* iterate from id #0 towards +inf */
            auto id_lane_iter1 = std::next(id_lane_iter0);
            for (auto iter = id_lane_iter1; iter != lane_section->id_to_lane.end(); iter++)
            {
                if (iter == id_lane_iter0)
                {
                    iter->second->outer_border = iter->second->lane_width;
                }
                else
                {
                    iter->second->inner_border = std::prev(iter)->second->outer_border;
                    iter->second->outer_border = std::prev(iter)->second->outer_border.add(iter->second->lane_width);
                }
            }

            /* iterate from id #0 towards -inf */
            std::map<int, std::shared_ptr<Lane>>::const_reverse_iterator r_id_lane_iter_1(id_lane_iter0);
            for (auto r_iter = r_id_lane_iter_1; r_iter != lane_section->id_to_lane.rend(); r_iter++)
            {
                if (r_iter == r_id_lane_iter_1)
                {
                    r_iter->second->outer_border = r_iter->second->lane_width.negate();
                }
                else
                {
                    r_iter->second->inner_border = std::prev(r_iter)->second->outer_border;
                    r_iter->second->outer_border = std::prev(r_iter)->second->outer_border.add(r_iter->second->lane_width.negate());
                }
            }

            for (auto& id_lane : lane_section->id_to_lane)
            {
                id_lane.second->inner_border = id_lane.second->inner_border.add(road->lane_offset);
                id_lane.second->outer_border = id_lane.second->outer_border.add(road->lane_offset);
            }
        }

        /* parse road objects */
        if (with_objects)
        {
            for (pugi::xml_node object_node : road_node.child("objects").children("object"))
            {
                std::shared_ptr<RoadObject> road_object = std::make_shared<RoadObject>();
                road_object->road = road;
                road_object->xml_node = object_node;

                road_object->type = object_node.attribute("type").as_string("");
                road_object->name = object_node.attribute("name").as_string("");
                road_object->id = object_node.attribute("id").as_string("");
                road_object->orientation = object_node.attribute("orientation").as_string("");

                road_object->s0 = object_node.attribute("s").as_double(0);
                road_object->t0 = object_node.attribute("t").as_double(0);
                road_object->z0 = object_node.attribute("zOffset").as_double(0);
                road_object->valid_length = object_node.attribute("validLength").as_double(0);
                road_object->length = object_node.attribute("length").as_double(0);
                road_object->width = object_node.attribute("width").as_double(0);
                road_object->radius = object_node.attribute("radius").as_double(0);
                road_object->height = object_node.attribute("height").as_double(0);
                road_object->hdg = object_node.attribute("hdg").as_double(0);
                road_object->pitch = object_node.attribute("pitch").as_double(0);
                road_object->roll = object_node.attribute("roll").as_double(0);

                CHECK_AND_REPAIR(road_object->s0 >= 0, "object::s < 0", road_object->s0 = 0);
                CHECK_AND_REPAIR(road_object->valid_length >= 0, "object::validLength < 0", road_object->valid_length = 0);
                CHECK_AND_REPAIR(road_object->length >= 0, "object::length < 0", road_object->length = 0);
                CHECK_AND_REPAIR(road_object->width >= 0, "object::width < 0", road_object->width = 0);
                CHECK_AND_REPAIR(road_object->radius >= 0, "object::radius < 0", road_object->radius = 0);

                for (pugi::xml_node repeat_node : object_node.children("repeat"))
                {
                    RoadObjectRepeat road_object_repeat;
                    road_object_repeat.s0 = repeat_node.attribute("s").as_double(NAN);
                    road_object_repeat.t_start = repeat_node.attribute("tStart").as_double(NAN);
                    road_object_repeat.t_end = repeat_node.attribute("tEnd").as_double(NAN);
                    road_object_repeat.width_start = repeat_node.attribute("widthStart").as_double(NAN);
                    road_object_repeat.width_end = repeat_node.attribute("widthEnd").as_double(NAN);
                    road_object_repeat.height_start = repeat_node.attribute("heightStart").as_double(NAN);
                    road_object_repeat.height_end = repeat_node.attribute("heightEnd").as_double(NAN);
                    road_object_repeat.z_offset_start = repeat_node.attribute("zOffsetStart").as_double(NAN);
                    road_object_repeat.z_offset_end = repeat_node.attribute("zOffsetEnd").as_double(NAN);
                    road_object_repeat.xml_node = repeat_node;

                    CHECK_AND_REPAIR(isnan(road_object_repeat.s0) || road_object_repeat.s0 >= 0, "object::repeat::s < 0", road_object_repeat.s0 = 0);
                    CHECK_AND_REPAIR(isnan(road_object_repeat.width_start) || road_object_repeat.width_start >= 0,
                                     "object::repeat::widthStart < 0",
                                     road_object_repeat.width_start = 0);
                    CHECK_AND_REPAIR(isnan(road_object_repeat.width_end) || road_object_repeat.width_end >= 0,
                                     "object::repeat::widthStart < 0",
                                     road_object_repeat.width_end = 0);

                    road_object_repeat.length = repeat_node.attribute("length").as_double(0);
                    road_object_repeat.distance = repeat_node.attribute("distance").as_double(0);

                    CHECK_AND_REPAIR(road_object_repeat.length >= 0, "object::repeat::length < 0", road_object_repeat.length = 0);
                    CHECK_AND_REPAIR(road_object_repeat.distance >= 0, "object::repeat::distance < 0", road_object_repeat.distance = 0);

                    road_object->repeats.push_back(std::move(road_object_repeat));
                }

                for (pugi::xml_node corner_local_node : object_node.child("outline").children("cornerLocal"))
                {
                    RoadObjectCorner road_object_corner_local;
                    road_object_corner_local.type = RoadObjectCorner::Type::Local;
                    road_object_corner_local.pt[0] = corner_local_node.attribute("u").as_double(0);
                    road_object_corner_local.pt[1] = corner_local_node.attribute("v").as_double(0);
                    road_object_corner_local.pt[2] = corner_local_node.attribute("z").as_double(0);
                    road_object_corner_local.height = corner_local_node.attribute("height").as_double(0);
                    road_object_corner_local.xml_node = corner_local_node;

                    road_object->outline.push_back(std::move(road_object_corner_local));
                }

                for (pugi::xml_node corner_road_node : object_node.child("outline").children("cornerRoad"))
                {
                    RoadObjectCorner road_object_corner_road;
                    road_object_corner_road.type = RoadObjectCorner::Type::Road;
                    road_object_corner_road.pt[0] = corner_road_node.attribute("s").as_double(0);
                    road_object_corner_road.pt[1] = corner_road_node.attribute("t").as_double(0);
                    road_object_corner_road.pt[2] = corner_road_node.attribute("dz").as_double(0);
                    road_object_corner_road.height = corner_road_node.attribute("height").as_double(0);
                    road_object_corner_road.xml_node = corner_road_node;

                    road_object->outline.push_back(std::move(road_object_corner_road));
                }

                CHECK_AND_REPAIR(road->id_to_object.find(road_object->id) == road->id_to_object.end(),
                                 (std::string("object::id already exists - ") + road_object->id).c_str(),
                                 road_object->id = road_object->id + std::string("_dup"));
                road->id_to_object[road_object->id] = road_object;
            }
        }
    }
}

ConstRoadSet OpenDriveMap::get_roads() const
{
    ConstRoadSet roads;
    for (const auto& id_road : this->roads)
        roads.insert(id_road.second);
    return roads;
}

RoadSet OpenDriveMap::get_roads()
{
    RoadSet roads;
    for (const auto& id_road : this->roads)
        roads.insert(id_road.second);
    return roads;
}

Mesh3D OpenDriveMap::get_refline_lines(double eps) const
{
    /* indices are pairs of vertices representing line segments */
    Mesh3D reflines;
    for (std::shared_ptr<const Road> road : this->get_roads())
    {
        const size_t idx_offset = reflines.vertices.size();

        std::set<double> s_vals = road->ref_line->approximate_linear(eps, 0.0, road->length);
        for (const double& s : s_vals)
        {
            reflines.vertices.push_back(road->ref_line->get_xyz(s));
            reflines.normals.push_back(normalize(road->ref_line->get_grad(s)));
        }

        for (size_t idx = idx_offset; idx < (idx_offset + s_vals.size() - 1); idx++)
        {
            reflines.indices.push_back(idx);
            reflines.indices.push_back(idx + 1);
        }
    }

    return reflines;
}

RoadNetworkMesh OpenDriveMap::get_mesh(double eps) const
{
    RoadNetworkMesh  out_mesh;
    LanesMesh&       lanes_mesh = out_mesh.lanes_mesh;
    RoadmarksMesh&   roadmarks_mesh = out_mesh.roadmarks_mesh;
    RoadObjectsMesh& road_objects_mesh = out_mesh.road_objects_mesh;

    for (std::shared_ptr<const Road> road : this->get_roads())
    {
        lanes_mesh.road_start_indices[lanes_mesh.vertices.size()] = road->id;
        roadmarks_mesh.road_start_indices[roadmarks_mesh.vertices.size()] = road->id;
        road_objects_mesh.road_start_indices[road_objects_mesh.vertices.size()] = road->id;

        for (std::shared_ptr<const LaneSection> lanesec : road->get_lanesections())
        {
            lanes_mesh.lanesec_start_indices[lanes_mesh.vertices.size()] = lanesec->s0;
            roadmarks_mesh.lanesec_start_indices[roadmarks_mesh.vertices.size()] = lanesec->s0;
            for (std::shared_ptr<const Lane> lane : lanesec->get_lanes())
            {
                const size_t lanes_idx_offset = lanes_mesh.vertices.size();
                lanes_mesh.lane_start_indices[lanes_idx_offset] = lane->id;
                lanes_mesh.add_mesh(lane->get_mesh(lanesec->s0, lanesec->get_end(), eps));

                size_t roadmarks_idx_offset = roadmarks_mesh.vertices.size();
                roadmarks_mesh.lane_start_indices[roadmarks_idx_offset] = lane->id;
                const std::vector<std::shared_ptr<RoadMark>> roadmarks = lane->get_roadmarks(lanesec->s0, lanesec->get_end());
                for (std::shared_ptr<const RoadMark> roadmark : roadmarks)
                {
                    roadmarks_idx_offset = roadmarks_mesh.vertices.size();
                    roadmarks_mesh.roadmark_type_start_indices[roadmarks_idx_offset] = roadmark->type;
                    roadmarks_mesh.add_mesh(lane->get_roadmark_mesh(roadmark, eps));
                }
            }
        }

        for (std::shared_ptr<const RoadObject> road_object : road->get_road_objects())
        {
            const size_t road_objs_idx_offset = road_objects_mesh.vertices.size();
            road_objects_mesh.road_object_start_indices[road_objs_idx_offset] = road_object->id;
            road_objects_mesh.add_mesh(road_object->get_mesh(eps));
        }
    }

    return out_mesh;
}

} // namespace odr
