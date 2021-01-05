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

#include "pugixml/pugixml.hpp"

#include <iostream>
#include <string>
#include <utility>

namespace odr
{
OpenDriveMap::OpenDriveMap(std::string xodr_file) : xodr_file(xodr_file)
{
    pugi::xml_document     doc;
    pugi::xml_parse_result result = doc.load_file(xodr_file.c_str());
    if (!result)
        printf("%s\n", result.description());

    pugi::xml_node odr_node = doc.child("OpenDRIVE");

    if (auto geoReference_node = odr_node.child("header").child("geoReference"))
        this->proj4 = geoReference_node.text().as_string();

    for (pugi::xml_node road_node : odr_node.children("road"))
    {
        /* make road */
        std::shared_ptr<Road> road = std::make_shared<Road>();
        road->length = road_node.attribute("length").as_double();
        road->id = road_node.attribute("id").as_string();
        road->junction = road_node.attribute("junction").as_string();
        road->name = road_node.attribute("name").as_string();

        this->roads[road->id] = road;

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
            }
        }

        /* parse road neighbors */
        for (pugi::xml_node road_neighbor_node : road_node.child("link").children("neighbor"))
        {
            RoadNeighbor road_neighbor;
            road_neighbor.elementId = road_neighbor_node.attribute("elementId").as_string();
            road_neighbor.side = road_neighbor_node.attribute("side").as_string();
            road_neighbor.direction = road_neighbor_node.attribute("direction").as_string();
            road->neighbors.push_back(road_neighbor);
        }

        /* parse road type and speed */
        for (pugi::xml_node road_type_node : road_node.children("type"))
        {
            double      s = road_type_node.attribute("s").as_double();
            std::string type = road_type_node.attribute("type").as_string();
            road->s0_to_type[s] = type;
            if (pugi::xml_node node = road_type_node.child("speed"))
            {
                SpeedRecord speed_record;
                speed_record.max = node.attribute("max").as_string();
                speed_record.unit = node.attribute("unit").as_string();
                road->s0_to_speed[s] = speed_record;
            }
        }

        /* make ref_line - parse road geometries */
        road->ref_line = std::make_shared<RefLine>(road->length);
        for (pugi::xml_node geometry_hdr_node : road_node.child("planView").children("geometry"))
        {
            double         s0 = geometry_hdr_node.attribute("s").as_double();
            double         x0 = geometry_hdr_node.attribute("x").as_double();
            double         y0 = geometry_hdr_node.attribute("y").as_double();
            double         hdg0 = geometry_hdr_node.attribute("hdg").as_double();
            double         length = geometry_hdr_node.attribute("length").as_double();
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
                if (geometry_node.attribute("pRange"))
                {
                    std::string pRange_str = geometry_node.attribute("pRange").as_string();
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
            }
        }

        std::map<std::string /*x path query*/, CubicSpline&> cubic_spline_fields{
            {".//elevationProfile//elevation", road->ref_line->elevation_profile},
            {".//lanes//laneOffset", road->lane_offset},
            {".//lateralProfile//superelevation", road->superelevation}};

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
                entry.second.s0_to_poly[s0] = std::make_shared<Poly3>(s0, a, b, c, d);
            }
        }

        /* parse crossfall - has extra attribute side */
        for (pugi::xml_node crossfall_node : road_node.child("lateralProfile").children("crossfall"))
        {
            double s0 = crossfall_node.attribute("s").as_double();
            double a = crossfall_node.attribute("a").as_double();
            double b = crossfall_node.attribute("b").as_double();
            double c = crossfall_node.attribute("c").as_double();
            double d = crossfall_node.attribute("d").as_double();

            road->crossfall.s0_to_poly[s0] = std::make_shared<Poly3>(s0, a, b, c, d);
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

        /* parse road lane sections and lanes */
        for (pugi::xml_node lane_section_node : road_node.child("lanes").children("laneSection"))
        {
            double s0 = lane_section_node.attribute("s").as_double();

            std::shared_ptr<LaneSection> lane_section = std::make_shared<LaneSection>(s0);
            lane_section->road = road;
            road->s0_to_lanesection[lane_section->s0] = lane_section;

            for (pugi::xpath_node lane_node : lane_section_node.select_nodes(".//lane"))
            {
                int         lane_id = lane_node.node().attribute("id").as_int();
                std::string lane_type = lane_node.node().attribute("type").as_string();
                bool        level = lane_node.node().attribute("level").as_bool();

                std::shared_ptr<Lane> lane = std::make_shared<Lane>(lane_id, level, lane_type);
                lane_section->id_to_lane[lane->id] = lane;

                for (pugi::xml_node lane_width_node : lane_node.node().children("width"))
                {
                    double s_offset = lane_width_node.attribute("sOffset").as_double();
                    double a = lane_width_node.attribute("a").as_double();
                    double b = lane_width_node.attribute("b").as_double();
                    double c = lane_width_node.attribute("c").as_double();
                    double d = lane_width_node.attribute("d").as_double();
                    lane->lane_width.s0_to_poly[s_offset] = std::make_shared<Poly3>(s_offset, a, b, c, d);
                }

                for (pugi::xml_node lane_height_node : lane_node.node().children("height"))
                {
                    double s_offset = lane_height_node.attribute("sOffset").as_double();
                    double inner = lane_height_node.attribute("inner").as_double();
                    double outer = lane_height_node.attribute("outer").as_double();
                    lane->s0_to_height_offset[s_offset] = HeightOffset{inner, outer};
                }

                for (pugi::xml_node roadmark_node : lane_node.node().children("roadMark"))
                {
                    double s_offset = roadmark_node.attribute("sOffset").as_double(0);
                    double width = roadmark_node.attribute("width").as_double(-1);
                    double height = roadmark_node.attribute("height").as_double(0);

                    std::string type = roadmark_node.attribute("type").as_string("none");
                    std::string weight = roadmark_node.attribute("weight").as_string("standard");
                    std::string color = roadmark_node.attribute("color").as_string("standard");
                    std::string material = roadmark_node.attribute("material").as_string("standard");
                    std::string laneChange = roadmark_node.attribute("laneChange").as_string("both");

                    std::vector<odr::RoadMarkLine> roadmark_lines;
                    if (pugi::xml_node roadmark_type_node = roadmark_node.child("type"))
                    {
                        std::string name = roadmark_type_node.attribute("name").as_string("");
                        double      line_width_1 = roadmark_type_node.attribute("width").as_double(-1);

                        for (pugi::xml_node roadmark_line : roadmark_type_node.children("line"))
                        {
                            double length = roadmark_line.attribute("length").as_double(0);
                            double space = roadmark_line.attribute("space").as_double(0);
                            double tOffset = roadmark_line.attribute("tOffset").as_double(0);
                            double sOffset = roadmark_line.attribute("sOffset").as_double(0);
                            double line_width_0 = roadmark_line.attribute("width").as_double(-1);
                            double line_width = line_width_0 < 0 ? line_width_1 : line_width_0;

                            std::string rule = roadmark_line.attribute("sOffset").as_string("none");
                            roadmark_lines.push_back({line_width, length, space, tOffset, sOffset, name, rule});
                        }
                    }
                    lane->s0_to_roadmark[s_offset] = RoadMark{width, height, type, weight, color, material, laneChange, roadmark_lines};
                }

                if (pugi::xml_node node = lane_node.node().child("link").child("predecessor"))
                    lane->predecessor = node.attribute("id").as_int();
                if (pugi::xml_node node = lane_node.node().child("link").child("successor"))
                    lane->successor = node.attribute("id").as_int();
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

} // namespace odr