#include "OpenDriveMap.h"
#include "Geometries/Geometries.h"
#include "Lanes.h"
#include "RefLine.h"
#include "Road.h"
#include "Utils.hpp"

#include "json11/json11.hpp"
#include "pugixml/pugixml.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>

namespace odr
{

OpenDriveMap::OpenDriveMap(std::string xodr_file)
    : xodr_file(xodr_file)
{
    pugi::xml_document     doc;
    pugi::xml_parse_result result = doc.load_file(xodr_file.c_str());
    if (!result)
    {
        std::cerr << result.description() << std::endl;
    }

    pugi::xpath_node_set roads = doc.select_nodes(".//road");
    for (pugi::xpath_node road_node : roads)
    {
        /* make road */
        const double road_length = road_node.node().attribute("length").as_double();
        const int    road_id = road_node.node().attribute("id").as_int();
        const int    junction_id = road_node.node().attribute("junction").as_int();

        std::shared_ptr<Road> road = std::make_shared<Road>(road_length, road_id, junction_id);
        this->roads[road->id] = road;

        /* make ref_line - parse road geometries */
        road->ref_line = std::make_shared<RefLine>();
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
                road->ref_line->geometries[s0] = std::make_shared<Line>(s0, x0, y0, hdg0, length, road);
            }
            else if (geometry_type == "spiral")
            {
                double curv_start = geometry_node.attribute("curvStart").as_double();
                double curv_end = geometry_node.attribute("curvEnd").as_double();
                road->ref_line->geometries[s0] = std::make_shared<Spiral>(s0, x0, y0, hdg0, length, curv_start, curv_end, road);
            }
            else if (geometry_type == "arc")
            {
                double curvature = geometry_node.attribute("curvature").as_double();
                road->ref_line->geometries[s0] = std::make_shared<Arc>(s0, x0, y0, hdg0, length, curvature, road);
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
                road->ref_line->geometries[s0] = std::make_shared<ParamPoly3>(s0, x0, y0, hdg0, length, aU, bU, cU, dU, aV, bV, cV, dV, road);
            }
            else
            {
                std::cerr << "Could not parse " << geometry_type << std::endl;
            }
        }

        /* parse elevation profiles */
        pugi::xpath_node_set elevation_nodes = road_node.node().select_nodes(".//elevationProfile//elevation");
        for (pugi::xpath_node elevation_node : elevation_nodes)
        {
            double s0 = elevation_node.node().attribute("s").as_double();
            double a = elevation_node.node().attribute("a").as_double();
            double b = elevation_node.node().attribute("b").as_double();
            double c = elevation_node.node().attribute("c").as_double();
            double d = elevation_node.node().attribute("d").as_double();
            road->ref_line->elevation_profiles[s0] = std::make_shared<ElevationProfile>(s0, a, b, c, d);
        }

        /* parse lane offsets */
        pugi::xpath_node_set lane_offset_nodes = road_node.node().select_nodes(".//lanes//laneOffset");
        for (pugi::xpath_node lane_offset_node : lane_offset_nodes)
        {
            double s0 = lane_offset_node.node().attribute("s").as_double();
            double a = lane_offset_node.node().attribute("a").as_double();
            double b = lane_offset_node.node().attribute("b").as_double();
            double c = lane_offset_node.node().attribute("c").as_double();
            double d = lane_offset_node.node().attribute("d").as_double();
            road->lane_offsets[s0] = std::make_shared<LaneOffset>(s0, a, b, c, d);
        }

        /* parse road lane sections and lanes */
        pugi::xpath_node_set lane_section_nodes = road_node.node().select_nodes(".//lanes//laneSection");
        for (pugi::xpath_node lane_section_node : lane_section_nodes)
        {
            double                       s0 = lane_section_node.node().attribute("s").as_double();
            std::shared_ptr<LaneSection> lane_section = std::make_shared<LaneSection>(s0);
            lane_section->road = road;
            road->lane_sections[lane_section->s0] = lane_section;
            for (pugi::xpath_node lane_node : lane_section_node.node().select_nodes(".//lane"))
            {
                int                                          lane_id = lane_node.node().attribute("id").as_int();
                std::string                                  lane_type = lane_node.node().attribute("type").as_string();
                std::map<double, std::shared_ptr<LaneWidth>> lane_widths;
                for (pugi::xpath_node lane_width_node : lane_node.node().select_nodes(".//width"))
                {
                    double s_offset = lane_width_node.node().attribute("sOffset").as_double();
                    double a = lane_width_node.node().attribute("a").as_double();
                    double b = lane_width_node.node().attribute("b").as_double();
                    double c = lane_width_node.node().attribute("c").as_double();
                    double d = lane_width_node.node().attribute("d").as_double();
                    lane_widths[s_offset] = std::make_shared<LaneWidth>(s_offset, a, b, c, d);
                }
                std::shared_ptr<Lane> lane = std::make_shared<Lane>(lane_id, lane_type, lane_widths);
                lane->lane_section = lane_section;
                lane_section->lanes[lane->id] = lane;
            }
        }
    }
}

} // namespace odr