#include "OpenDriveMap.h"
#include "Geometries/Arc.h"
#include "Geometries/CubicSpline.h"
#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"
#include "Geometries/Spiral.h"
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
        std::cerr << result.description() << std::endl;

    this->proj4 = doc.select_node("/OpenDRIVE/header/geoReference").node().text().as_string();

    pugi::xpath_node_set roads = doc.select_nodes("/OpenDRIVE/road");
    for (pugi::xpath_node road_node : roads)
    {
        /* make road */
        const double road_length = road_node.node().attribute("length").as_double();
        const int    road_id = road_node.node().attribute("id").as_int();
        const int    junction_id = road_node.node().attribute("junction").as_int();

        std::shared_ptr<Road> road = std::make_shared<Road>(road_length, road_id, junction_id);
        this->roads[road->id] = road;

        /* parse road links */
        for (bool is_predecessor : {true, false})
        {
            const std::string xpath = is_predecessor ? ".//link//predecessor" : ".//link//successor";
            if (pugi::xml_node node = doc.select_node(xpath.c_str()).node())
            {
                RoadLink& link = is_predecessor ? road->predecessor : road->successor;
                link.elementId = node.child("elementId").text().as_int();
                link.elementType = node.child("elementType").text().as_string();
                link.contactPoint = node.child("contactPoint").text().as_string();
            }
        }

        /* parse road neighbors */
        pugi::xpath_node_set road_neighbor_nodes = road_node.node().select_nodes(".//link//neighbor");
        for (pugi::xpath_node road_neighbor_node : road_neighbor_nodes)
        {
            RoadNeighbor road_neighbor;
            road_neighbor.elementId = road_neighbor_node.node().attribute("elementId").as_int();
            road_neighbor.side = road_neighbor_node.node().attribute("side").as_string();
            road_neighbor.direction = road_neighbor_node.node().attribute("direction").as_string();
            road->neighbors.push_back(road_neighbor);
        }

        /* parse road type and speed */
        pugi::xpath_node_set road_type_nodes = road_node.node().select_nodes(".//type");
        for (pugi::xpath_node road_type_node : road_type_nodes)
        {
            double      s = road_type_node.node().attribute("s").as_double();
            std::string type = road_type_node.node().attribute("type").as_string();
            road->s0_to_type[s] = type;
            if (pugi::xml_node node = road_type_node.node().child("speed"))
            {
                SpeedRecord speed_record;
                speed_record.max = node.attribute("max").as_double();
                speed_record.unit = node.attribute("unit").as_string();
                road->s0_to_speed[s] = speed_record;
            }
        }

        /* make ref_line - parse road geometries */
        road->ref_line = std::make_shared<RefLine>(road_length);
        pugi::xpath_node_set geometry_headers = road_node.node().select_nodes(".//planView//geometry");
        for (pugi::xpath_node geometry_hdr_node : geometry_headers)
        {
            double         s0 = geometry_hdr_node.node().attribute("s").as_double();
            double         x0 = geometry_hdr_node.node().attribute("x").as_double();
            double         y0 = geometry_hdr_node.node().attribute("y").as_double();
            double         hdg0 = geometry_hdr_node.node().attribute("hdg").as_double();
            double         length = geometry_hdr_node.node().attribute("length").as_double();
            pugi::xml_node geometry_node = geometry_hdr_node.node().first_child();
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
                road->ref_line->s0_to_geometry[s0] = std::make_shared<ParamPoly3>(s0, x0, y0, hdg0, length, aU, bU, cU, dU, aV, bV, cV, dV);
            }
            else
            {
                std::cerr << "Could not parse " << geometry_type << std::endl;
            }
        }

        std::map<std::string /*x path query*/, CubicSpline&> cubic_spline_fields{
            {".//elevationProfile//elevation", road->ref_line->elevation_profile},
            {".//lanes//laneOffset", road->lane_offset},
            {".//lateralProfile//superelevation", road->superelevation}};

        /* parse elevation profiles, lane offsets, superelevation */
        for (auto entry : cubic_spline_fields)
        {
            pugi::xpath_node_set nodes = road_node.node().select_nodes(entry.first.c_str());
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
        pugi::xpath_node_set crossfall_nodes = road_node.node().select_nodes(".//lateralProfile//crossfall");
        for (pugi::xpath_node crossfall_node : crossfall_nodes)
        {
            double s0 = crossfall_node.node().attribute("s").as_double();
            double a = crossfall_node.node().attribute("a").as_double();
            double b = crossfall_node.node().attribute("b").as_double();
            double c = crossfall_node.node().attribute("c").as_double();
            double d = crossfall_node.node().attribute("d").as_double();

            road->crossfall.s0_to_poly[s0] = std::make_shared<Poly3>(s0, a, b, c, d);
            if (pugi::xml_attribute side = crossfall_node.node().attribute("side"))
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
        pugi::xpath_node_set lateral_profile_shape_nodes = road_node.node().select_nodes(".//lateralProfile//shape");
        if (lateral_profile_shape_nodes.size())
            std::cerr << "Lateral Profile Shape not supported" << std::endl;

        /* parse road lane sections and lanes */
        pugi::xpath_node_set lane_section_nodes = road_node.node().select_nodes(".//lanes//laneSection");
        for (pugi::xpath_node lane_section_node : lane_section_nodes)
        {
            double s0 = lane_section_node.node().attribute("s").as_double();

            std::shared_ptr<LaneSection> lane_section = std::make_shared<LaneSection>(s0);
            lane_section->road = road;
            road->s0_to_lanesection[lane_section->s0] = lane_section;

            for (pugi::xpath_node lane_node : lane_section_node.node().select_nodes(".//lane"))
            {
                int         lane_id = lane_node.node().attribute("id").as_int();
                std::string lane_type = lane_node.node().attribute("type").as_string();
                bool        level = lane_node.node().attribute("level").as_bool();

                std::shared_ptr<Lane> lane = std::make_shared<Lane>(lane_id, level, lane_type);
                lane_section->id_to_lane[lane->id] = lane;

                for (pugi::xpath_node lane_width_node : lane_node.node().select_nodes(".//width"))
                {
                    double s_offset = lane_width_node.node().attribute("sOffset").as_double();
                    double a = lane_width_node.node().attribute("a").as_double();
                    double b = lane_width_node.node().attribute("b").as_double();
                    double c = lane_width_node.node().attribute("c").as_double();
                    double d = lane_width_node.node().attribute("d").as_double();
                    lane->lane_width.s0_to_poly[s_offset] = std::make_shared<Poly3>(s_offset, a, b, c, d);
                }

                for (pugi::xpath_node lane_height_node : lane_node.node().select_nodes(".//height"))
                {
                    double s_offset = lane_height_node.node().attribute("sOffset").as_double();
                    double inner = lane_height_node.node().attribute("inner").as_double();
                    double outer = lane_height_node.node().attribute("outer").as_double();
                    lane->s0_to_height_offset[s_offset] = HeightOffset{inner, outer};
                }

                if (pugi::xml_node node = lane_node.node().select_node(".//link/predecessor").node())
                    lane->predecessor = node.attribute("id").as_int();
                if (pugi::xml_node node = lane_node.node().select_node(".//link/successor").node())
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