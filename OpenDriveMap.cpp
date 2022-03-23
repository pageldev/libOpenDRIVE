#include "OpenDriveMap.h"
#include "Geometries/Arc.h"
#include "Geometries/CubicSpline.h"
#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"
#include "Geometries/RoadGeometry.h"
#include "Geometries/Spiral.h"
#include "Junction.h"
#include "LaneSection.h"
#include "Lanes.h"
#include "Math.hpp"
#include "RefLine.h"
#include "Road.h"
#include "RoadMark.h"
#include "RoadObject.h"
#include "Utils.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <type_traits>
#include <vector>

namespace odr
{
OpenDriveMap::OpenDriveMap(const std::string& xodr_file, const OpenDriveMapConfig& config) : xodr_file(xodr_file)
{
    pugi::xml_parse_result result = this->xml_doc.load_file(xodr_file.c_str());
    if (!result)
        printf("%s\n", result.description());

    pugi::xml_node odr_node = this->xml_doc.child("OpenDRIVE");

    if (auto geoReference_node = odr_node.child("header").child("geoReference"))
        this->proj4 = geoReference_node.text().as_string("");

    size_t cnt = 1;
    if (config.center_map)
    {
        for (pugi::xml_node road_node : odr_node.children("road"))
        {
            for (pugi::xml_node geometry_hdr_node : road_node.child("planView").children("geometry"))
            {
                const double x0 = geometry_hdr_node.attribute("x").as_double(0.0);
                this->x_offs = this->x_offs + ((x0 - this->x_offs) / cnt);
                const double y0 = geometry_hdr_node.attribute("y").as_double(0.0);
                this->y_offs = this->y_offs + ((y0 - this->y_offs) / cnt);
                cnt++;
            }
        }
    }

    for (pugi::xml_node junction_node : odr_node.children("junction"))
    {
        /* make junction */
        std::shared_ptr<Junction> junction = std::make_shared<Junction>();
        junction->id = junction_node.attribute("id").as_string("");
        junction->name = junction_node.attribute("name").as_string("");
        junction->xml_node = junction_node;

        for (pugi::xml_node connection_node : junction_node.children("connection"))
        {
            JunctionConnection junction_connection;
            junction_connection.id = connection_node.attribute("id").as_string("");
            junction_connection.incoming_road = connection_node.attribute("incomingRoad").as_string("");
            junction_connection.connecting_road = connection_node.attribute("connectingRoad").as_string("");

            std::string contact_point_str = connection_node.attribute("contactPoint").as_string("");
            CHECK_AND_REPAIR(contact_point_str == "start" || contact_point_str == "end",
                             "Junction::Connection::contactPoint invalid value",
                             contact_point_str = "start"); // default to start
            junction_connection.contact_point =
                (contact_point_str == "start") ? JunctionConnection::ContactPoint::Start : JunctionConnection::ContactPoint::End;

            for (pugi::xml_node lane_link_node : connection_node.children("laneLink"))
            {
                JunctionLaneLink lane_link;
                lane_link.from = lane_link_node.attribute("from").as_int(0);
                lane_link.to = lane_link_node.attribute("to").as_int(0);
                junction_connection.lane_links.push_back(lane_link);
            }

            junction->connections[junction_connection.id] = junction_connection;
        }

        const size_t num_conns = junction->connections.size();
        CHECK(num_conns > 0, "Junction::connections == 0");
        if (num_conns < 1)
            continue;

        for (pugi::xml_node priority_node : junction_node.children("priority"))
        {
            JunctionPriority junction_priority;
            junction_priority.high = priority_node.attribute("high").as_string("");
            junction_priority.low = priority_node.attribute("low").as_string("");
            junction->priorities.push_back(junction_priority);
        }

        for (pugi::xml_node controller_node : junction_node.children("controller"))
        {
            JunctionController junction_controller;
            junction_controller.id = controller_node.attribute("id").as_string("");
            junction_controller.type = controller_node.attribute("type").as_string("");
            junction_controller.sequence = controller_node.attribute("sequence").as_uint(0);
            junction->controllers[junction_controller.id] = junction_controller;
        }

        this->junctions[junction->id] = junction;
    }

    for (pugi::xml_node road_node : odr_node.children("road"))
    {
        /* make road */
        std::shared_ptr<Road> road = std::make_shared<Road>();
        road->length = road_node.attribute("length").as_double(0.0);
        road->id = road_node.attribute("id").as_string("");
        road->junction = road_node.attribute("junction").as_string("");
        road->name = road_node.attribute("name").as_string("");
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
                link.id = road_link_node.attribute("elementId").as_string("");

                std::string type_str = road_link_node.attribute("elementType").as_string("");
                CHECK_AND_REPAIR(type_str == "road" || type_str == "junction",
                                 "Road::Succ/Predecessor::Link::elementType invalid type",
                                 type_str = "road"); // default to road
                link.type = (type_str == "road") ? RoadLink::Type::Road : RoadLink::Type::Junction;

                if (link.type == RoadLink::Type::Road)
                {
                    // junction connection has no contact point
                    std::string contact_point_str = road_link_node.attribute("contactPoint").as_string("");
                    CHECK_AND_REPAIR(contact_point_str == "start" || contact_point_str == "end",
                                     "Road::Succ/Predecessor::Link::contactPoint invalid type",
                                     contact_point_str = "start"); // default to start
                    link.contact_point = (contact_point_str == "start") ? RoadLink::ContactPoint::Start : RoadLink::ContactPoint::End;
                }

                link.xml_node = road_link_node;
            }
        }

        /* parse road neighbors */
        for (pugi::xml_node road_neighbor_node : road_node.child("link").children("neighbor"))
        {
            RoadNeighbor road_neighbor;
            road_neighbor.id = road_neighbor_node.attribute("elementId").as_string("");
            road_neighbor.side = road_neighbor_node.attribute("side").as_string("");
            road_neighbor.direction = road_neighbor_node.attribute("direction").as_string("");
            road_neighbor.xml_node = road_neighbor_node;
            road->neighbors.push_back(road_neighbor);
        }

        /* parse road type and speed */
        for (pugi::xml_node road_type_node : road_node.children("type"))
        {
            double      s = road_type_node.attribute("s").as_double(0.0);
            std::string type = road_type_node.attribute("type").as_string("");

            CHECK_AND_REPAIR(s >= 0, "road::type::s < 0", s = 0);

            road->s_to_type[s] = type;
            if (pugi::xml_node node = road_type_node.child("speed"))
            {
                SpeedRecord speed_record;
                speed_record.max = node.attribute("max").as_string("");
                speed_record.unit = node.attribute("unit").as_string("");
                speed_record.xml_node = node;
                road->s_to_speed[s] = speed_record;
            }
        }

        /* make ref_line - parse road geometries */
        road->ref_line = std::make_shared<RefLine>(road->length);
        for (pugi::xml_node geometry_hdr_node : road_node.child("planView").children("geometry"))
        {
            double s0 = geometry_hdr_node.attribute("s").as_double(0.0);
            double x0 = geometry_hdr_node.attribute("x").as_double(0.0) - this->x_offs;
            double y0 = geometry_hdr_node.attribute("y").as_double(0.0) - this->y_offs;
            double hdg0 = geometry_hdr_node.attribute("hdg").as_double(0.0);
            double length = geometry_hdr_node.attribute("length").as_double(0.0);

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
                double curv_start = geometry_node.attribute("curvStart").as_double(0.0);
                double curv_end = geometry_node.attribute("curvEnd").as_double(0.0);
                road->ref_line->s0_to_geometry[s0] = std::make_shared<Spiral>(s0, x0, y0, hdg0, length, curv_start, curv_end);
            }
            else if (geometry_type == "arc")
            {
                double curvature = geometry_node.attribute("curvature").as_double(0.0);
                road->ref_line->s0_to_geometry[s0] = std::make_shared<Arc>(s0, x0, y0, hdg0, length, curvature);
            }
            else if (geometry_type == "paramPoly3")
            {
                double aU = geometry_node.attribute("aU").as_double(0.0);
                double bU = geometry_node.attribute("bU").as_double(0.0);
                double cU = geometry_node.attribute("cU").as_double(0.0);
                double dU = geometry_node.attribute("dU").as_double(0.0);
                double aV = geometry_node.attribute("aV").as_double(0.0);
                double bV = geometry_node.attribute("bV").as_double(0.0);
                double cV = geometry_node.attribute("cV").as_double(0.0);
                double dV = geometry_node.attribute("dV").as_double(0.0);

                bool pRange_normalized = true;
                if (geometry_node.attribute("pRange") || geometry_hdr_node.attribute("pRange"))
                {
                    std::string pRange_str = geometry_node.attribute("pRange") ? geometry_node.attribute("pRange").as_string("")
                                                                               : geometry_hdr_node.attribute("pRange").as_string("");
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

        if (config.with_lateralProfile)
            cubic_spline_fields.insert({".//lateralProfile//superelevation", road->superelevation});

        /* parse elevation profiles, lane offsets, superelevation */
        for (auto entry : cubic_spline_fields)
        {
            pugi::xpath_node_set nodes = road_node.select_nodes(entry.first.c_str());
            for (pugi::xpath_node node : nodes)
            {
                double s0 = node.node().attribute("s").as_double(0.0);
                double a = node.node().attribute("a").as_double(0.0);
                double b = node.node().attribute("b").as_double(0.0);
                double c = node.node().attribute("c").as_double(0.0);
                double d = node.node().attribute("d").as_double(0.0);

                CHECK_AND_REPAIR(s0 >= 0, (entry.first + "::s < 0").c_str(), s0 = 0);

                entry.second.s0_to_poly[s0] = Poly3(s0, a, b, c, d);
            }
        }

        /* parse crossfall - has extra attribute side */
        if (config.with_lateralProfile)
        {
            for (pugi::xml_node crossfall_node : road_node.child("lateralProfile").children("crossfall"))
            {
                double s0 = crossfall_node.attribute("s").as_double(0.0);
                double a = crossfall_node.attribute("a").as_double(0.0);
                double b = crossfall_node.attribute("b").as_double(0.0);
                double c = crossfall_node.attribute("c").as_double(0.0);
                double d = crossfall_node.attribute("d").as_double(0.0);

                CHECK_AND_REPAIR(s0 >= 0, "road::lateralProfile::crossfall::s < 0", s0 = 0);

                Poly3 crossfall_poly(s0, a, b, c, d);
                road->crossfall.s0_to_poly[s0] = crossfall_poly;
                if (pugi::xml_attribute side = crossfall_node.attribute("side"))
                {
                    std::string side_str = side.as_string("");
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
            double s0 = lane_section_node.attribute("s").as_double(0.0);

            std::shared_ptr<LaneSection> lane_section = std::make_shared<LaneSection>(s0);
            lane_section->road = road;
            lane_section->xml_node = lane_section_node;
            road->s_to_lanesection[lane_section->s0] = lane_section;

            for (pugi::xpath_node lane_xpath_node : lane_section_node.select_nodes(".//lane"))
            {
                pugi::xml_node lane_node = lane_xpath_node.node();
                int            lane_id = lane_node.attribute("id").as_int(0);
                std::string    lane_type = lane_node.attribute("type").as_string("");
                bool           level = lane_node.attribute("level").as_bool(false);

                std::shared_ptr<Lane> lane = std::make_shared<Lane>(lane_id, level, lane_type);
                lane->road = road;
                lane->lane_section = lane_section;
                lane->xml_node = lane_node;
                lane_section->id_to_lane[lane->id] = lane;

                for (pugi::xml_node lane_width_node : lane_node.children("width"))
                {
                    double s_offset = lane_width_node.attribute("sOffset").as_double(0.0);
                    double a = lane_width_node.attribute("a").as_double(0.0);
                    double b = lane_width_node.attribute("b").as_double(0.0);
                    double c = lane_width_node.attribute("c").as_double(0.0);
                    double d = lane_width_node.attribute("d").as_double(0.0);

                    CHECK_AND_REPAIR(s_offset >= 0, "lane::width::sOffset < 0", s_offset = 0);
                    lane->lane_width.s0_to_poly[s0 + s_offset] = Poly3(s0 + s_offset, a, b, c, d);
                }

                if (config.with_laneHeight)
                {
                    for (pugi::xml_node lane_height_node : lane_node.children("height"))
                    {
                        double s_offset = lane_height_node.attribute("sOffset").as_double(0.0);
                        double inner = lane_height_node.attribute("inner").as_double(0.0);
                        double outer = lane_height_node.attribute("outer").as_double(0.0);

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
                    lane->predecessor = node.attribute("id").as_int(0);
                if (pugi::xml_node node = lane_node.child("link").child("successor"))
                    lane->successor = node.attribute("id").as_int(0);
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
        if (config.with_road_objects)
        {
            const RoadObjectCorner::Type default_local_outline_type =
                config.abs_z_for_for_local_road_obj_outline ? RoadObjectCorner::Type::Local_AbsZ : RoadObjectCorner::Type::Local_RelZ;

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

                    CHECK_AND_REPAIR(
                        std::isnan(road_object_repeat.s0) || road_object_repeat.s0 >= 0, "object::repeat::s < 0", road_object_repeat.s0 = 0);
                    CHECK_AND_REPAIR(std::isnan(road_object_repeat.width_start) || road_object_repeat.width_start >= 0,
                                     "object::repeat::widthStart < 0",
                                     road_object_repeat.width_start = 0);
                    CHECK_AND_REPAIR(std::isnan(road_object_repeat.width_end) || road_object_repeat.width_end >= 0,
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
                    road_object_corner_local.type = default_local_outline_type;
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

RoutingGraph OpenDriveMap::get_routing_graph() const
{
    RoutingGraph routing_graph;
    using RoadPtr = std::shared_ptr<Road>;
    using LanePtr = std::shared_ptr<Lane>;
    using LanesecPtr = std::shared_ptr<LaneSection>;

    for (const bool find_successor : {true, false})
    {
        for (const auto& id_road : this->roads)
        {
            RoadPtr         road = id_road.second;
            const RoadLink& road_link = find_successor ? road->successor : road->predecessor;
            if (road_link.type != RoadLink::Type::Road || road_link.contact_point == RoadLink::ContactPoint::None)
                continue;

            RoadPtr next_road = try_get_val(this->roads, road_link.id, RoadPtr(nullptr));
            if (!next_road)
                continue;

            LanesecPtr next_road_lanesec = (road_link.contact_point == RoadLink::ContactPoint::Start) ? next_road->s_to_lanesection.begin()->second
                                                                                                      : next_road->s_to_lanesection.rbegin()->second;
            for (auto s_lanesec_iter = road->s_to_lanesection.begin(); s_lanesec_iter != road->s_to_lanesection.end(); s_lanesec_iter++)
            {
                LanesecPtr lanesec = s_lanesec_iter->second;
                LanesecPtr next_lanesec = nullptr;
                if (find_successor && std::next(s_lanesec_iter) == road->s_to_lanesection.end())
                    next_lanesec = next_road_lanesec; // take next road to find successor
                else if (!find_successor && s_lanesec_iter == road->s_to_lanesection.begin())
                    next_lanesec = next_road_lanesec; // take prev. road to find predecessor
                else
                    next_lanesec = find_successor ? std::next(s_lanesec_iter)->second : std::prev(s_lanesec_iter)->second;

                for (const auto& id_lane : lanesec->id_to_lane)
                {
                    LanePtr   lane = id_lane.second;
                    const int next_lane_id = find_successor ? lane->successor : lane->predecessor;
                    if (next_lane_id == 0)
                        continue;

                    LanePtr next_lane = try_get_val(next_lanesec->id_to_lane, next_lane_id, LanePtr(nullptr));
                    if (!next_lane)
                        continue;

                    LanePtr from_lane = find_successor ? lane : next_lane;
                    LanePtr to_lane = find_successor ? next_lane : lane;

                    const RoutingGraphVertex from(from_lane->road.lock()->id, from_lane->lane_section.lock()->s0, from_lane->id);
                    const RoutingGraphVertex to(to_lane->road.lock()->id, to_lane->lane_section.lock()->s0, to_lane->id);
                    routing_graph.add_edge(RoutingGraphEdge(from, to));
                }
            }
        }
    }

    for (const auto& id_junc : this->junctions)
    {
        for (const auto& id_conn : id_junc.second->connections)
        {
            const JunctionConnection& conn = id_conn.second;

            RoadPtr incoming_road = try_get_val(this->roads, conn.incoming_road, RoadPtr(nullptr));
            RoadPtr connecting_road = try_get_val(this->roads, conn.connecting_road, RoadPtr(nullptr));
            if (!incoming_road || !connecting_road)
                continue;

            const bool is_succ_junc = incoming_road->successor.type == RoadLink::Type::Junction && incoming_road->successor.id == conn.id;
            const bool is_pred_junc = incoming_road->predecessor.type == RoadLink::Type::Junction && incoming_road->predecessor.id == conn.id;
            if (!is_succ_junc && !is_pred_junc)
                continue;

            LanesecPtr incoming_lanesec =
                is_succ_junc ? incoming_road->s_to_lanesection.rbegin()->second : incoming_road->s_to_lanesection.begin()->second;
            LanesecPtr connecting_lanesec = (conn.contact_point == JunctionConnection::ContactPoint::Start)
                                                ? connecting_road->s_to_lanesection.begin()->second
                                                : connecting_road->s_to_lanesection.rbegin()->second;
            for (const JunctionLaneLink& lane_link : conn.lane_links)
            {
                if (lane_link.from == 0 || lane_link.to == 0)
                    continue;
                LanePtr from_lane = try_get_val(incoming_lanesec->id_to_lane, lane_link.from, LanePtr(nullptr));
                LanePtr to_lane = try_get_val(connecting_lanesec->id_to_lane, lane_link.to, LanePtr(nullptr));
                if (!from_lane || !to_lane)
                    continue;

                const RoutingGraphVertex from(from_lane->road.lock()->id, from_lane->lane_section.lock()->s0, from_lane->id);
                const RoutingGraphVertex to(to_lane->road.lock()->id, to_lane->lane_section.lock()->s0, to_lane->id);
                routing_graph.add_edge(RoutingGraphEdge(from, to));
            }
        }
    }

    return routing_graph;
}

} // namespace odr
