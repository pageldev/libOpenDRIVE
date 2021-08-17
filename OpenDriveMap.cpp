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
OpenDriveMap::OpenDriveMap(std::string xodr_file, bool with_lateralProfile, bool with_laneHeight, bool center_map) : xodr_file(xodr_file)
{
    pugi::xml_document     doc;
    pugi::xml_parse_result result = doc.load_file(xodr_file.c_str());
    if (!result)
        printf("%s\n", result.description());

    pugi::xml_node odr_node = doc.child("OpenDRIVE");

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
            road->s_to_type[s] = type;
            if (pugi::xml_node node = road_type_node.child("speed"))
            {
                SpeedRecord speed_record;
                speed_record.max = node.attribute("max").as_string();
                speed_record.unit = node.attribute("unit").as_string();
                road->s_to_speed[s] = speed_record;
            }
        }

        /* make ref_line - parse road geometries */
        road->ref_line = std::make_shared<RefLine>(road->length);
        for (pugi::xml_node geometry_hdr_node : road_node.child("planView").children("geometry"))
        {
            double         s0 = geometry_hdr_node.attribute("s").as_double();
            double         x0 = geometry_hdr_node.attribute("x").as_double() - this->x_offs;
            double         y0 = geometry_hdr_node.attribute("y").as_double() - this->y_offs;
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
            road->s_to_lanesection[lane_section->s0] = lane_section;

            for (pugi::xpath_node lane_node : lane_section_node.select_nodes(".//lane"))
            {
                int         lane_id = lane_node.node().attribute("id").as_int();
                std::string lane_type = lane_node.node().attribute("type").as_string();
                bool        level = lane_node.node().attribute("level").as_bool();

                std::shared_ptr<Lane> lane = std::make_shared<Lane>(lane_id, level, lane_type);
                lane->road = road;
                lane_section->id_to_lane[lane->id] = lane;

                for (pugi::xml_node lane_width_node : lane_node.node().children("width"))
                {
                    double sOffs = lane_width_node.attribute("sOffset").as_double();
                    double a = lane_width_node.attribute("a").as_double();
                    double b = lane_width_node.attribute("b").as_double();
                    double c = lane_width_node.attribute("c").as_double();
                    double d = lane_width_node.attribute("d").as_double();
                    lane->lane_width.s0_to_poly[s0 + sOffs] = Poly3(s0 + sOffs, a, b, c, d);
                }

                if (with_laneHeight)
                {
                    for (pugi::xml_node lane_height_node : lane_node.node().children("height"))
                    {
                        double s_offset = lane_height_node.attribute("sOffset").as_double();
                        double inner = lane_height_node.attribute("inner").as_double();
                        double outer = lane_height_node.attribute("outer").as_double();
                        lane->s_to_height_offset[s0 + s_offset] = HeightOffset{inner, outer};
                    }
                }

                for (pugi::xml_node roadmark_node : lane_node.node().children("roadMark"))
                {
                    double sOffsetRoadMark = roadmark_node.attribute("sOffset").as_double(0);
                    double width = roadmark_node.attribute("width").as_double(-1);
                    double height = roadmark_node.attribute("height").as_double(0);

                    std::string type = roadmark_node.attribute("type").as_string("none");
                    std::string weight = roadmark_node.attribute("weight").as_string("standard");
                    std::string color = roadmark_node.attribute("color").as_string("standard");
                    std::string material = roadmark_node.attribute("material").as_string("standard");
                    std::string laneChange = roadmark_node.attribute("laneChange").as_string("both");

                    RoadMarkGroup roadmark_group{width, height, sOffsetRoadMark, type, weight, color, material, laneChange};

                    if (pugi::xml_node roadmark_type_node = roadmark_node.child("type"))
                    {
                        std::string name = roadmark_type_node.attribute("name").as_string("");
                        double      line_width_1 = roadmark_type_node.attribute("width").as_double(-1);

                        for (pugi::xml_node roadmarks_line_node : roadmark_type_node.children("line"))
                        {
                            double length = roadmarks_line_node.attribute("length").as_double(0);
                            double space = roadmarks_line_node.attribute("space").as_double(0);
                            double tOffset = roadmarks_line_node.attribute("tOffset").as_double(0);
                            double sOffsetRoadMarksLine = roadmarks_line_node.attribute("sOffset").as_double(0);
                            double line_width_0 = roadmarks_line_node.attribute("width").as_double(-1);
                            double line_width = line_width_0 < 0 ? line_width_1 : line_width_0;

                            std::string rule = roadmarks_line_node.attribute("rule").as_string("none");

                            RoadMarksLine roadmarks_line{line_width, length, space, tOffset, sOffsetRoadMarksLine, name, rule};
                            roadmark_group.s_to_roadmarks_line[s0 + sOffsetRoadMark + sOffsetRoadMarksLine] = roadmarks_line;
                        }
                    }
                    lane->s_to_roadmark_group[s0 + sOffsetRoadMark] = roadmark_group;
                }

                if (pugi::xml_node node = lane_node.node().child("link").child("predecessor"))
                    lane->predecessor = node.attribute("id").as_int();
                if (pugi::xml_node node = lane_node.node().child("link").child("successor"))
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
        const Line3D refl_pts = road->ref_line->get_line(0.0, road->length, eps);
        reflines.vertices.insert(reflines.vertices.end(), refl_pts.begin(), refl_pts.end());
        for (size_t idx = idx_offset; idx < (idx_offset + refl_pts.size() - 1); idx++)
        {
            reflines.indices.push_back(idx);
            reflines.indices.push_back(idx + 1);
        }
    }

    return reflines;
}

RoadNetworkMesh OpenDriveMap::get_mesh(double eps) const
{
    RoadNetworkMesh    out_mesh;
    LaneMeshUnion&     lanes_union = out_mesh.lane_mesh_union;
    RoadmarkMeshUnion& roadmarks_union = out_mesh.roadmark_mesh_union;
    for (std::shared_ptr<const Road> road : this->get_roads())
    {
        lanes_union.road_start_indices[lanes_union.vertices.size()] = road->id;
        roadmarks_union.road_start_indices[roadmarks_union.vertices.size()] = road->id;
        for (std::shared_ptr<const LaneSection> lanesec : road->get_lanesections())
        {
            lanes_union.lanesec_start_indices[lanes_union.vertices.size()] = lanesec->s0;
            roadmarks_union.lanesec_start_indices[roadmarks_union.vertices.size()] = lanesec->s0;
            for (std::shared_ptr<const Lane> lane : lanesec->get_lanes())
            {
                size_t idx_offset = lanes_union.vertices.size();
                lanes_union.lane_start_indices[idx_offset] = lane->id;
                Mesh3D lane_mesh = lane->get_mesh(lanesec->s0, lanesec->get_end(), eps);
                lanes_union.st_coordinates.insert(lanes_union.st_coordinates.end(), lane_mesh.st_coordinates.begin(), lane_mesh.st_coordinates.end());
                lanes_union.vertices.insert(lanes_union.vertices.end(), lane_mesh.vertices.begin(), lane_mesh.vertices.end());
                for (const size_t& idx : lane_mesh.indices)
                    lanes_union.indices.push_back(idx + idx_offset);

                idx_offset = roadmarks_union.vertices.size();
                roadmarks_union.lane_start_indices[idx_offset] = lane->id;
                const std::vector<RoadMark> roadmarks = lane->get_roadmarks(lanesec->s0, lanesec->get_end());
                for (const RoadMark& roadmark : roadmarks)
                {
                    idx_offset = roadmarks_union.vertices.size();
                    const Mesh3D roadmark_mesh = lane->get_roadmark_mesh(roadmark, eps);
                    roadmarks_union.vertices.insert(roadmarks_union.vertices.end(), roadmark_mesh.vertices.begin(), roadmark_mesh.vertices.end());
                    for (const size_t& idx : roadmark_mesh.indices)
                        roadmarks_union.indices.push_back(idx + idx_offset);
                    roadmarks_union.roadmark_type_start_indices[idx_offset] = roadmark.type;
                }
            }
        }
    }

    return out_mesh;
}

} // namespace odr
