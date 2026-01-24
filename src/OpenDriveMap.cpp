#include "OpenDriveMap.h"
#include "Geometries/Arc.h"
#include "Geometries/CubicSpline.h"
#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"
#include "Geometries/RoadGeometry.h"
#include "Geometries/Spiral.h"
#include "Junction.h"
#include "Lane.h"
#include "LaneSection.h"
#include "LaneValidityRecord.h"
#include "Log.hpp"
#include "Math.hpp"
#include "RefLine.h"
#include "Road.h"
#include "RoadMark.h"
#include "RoadObject.h"
#include "RoadSignal.h"
#include "Utils.hpp"

#include <algorithm>
#include <cctype>
#include <climits>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iterator>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace odr
{

OpenDriveMap::OpenDriveMap(const std::string& xodr_file,
                           const bool         center_map,
                           const bool         with_road_objects,
                           const bool         with_lateral_profile,
                           const bool         with_lane_height,
                           const bool         abs_z_for_for_local_road_obj_outline,
                           const bool         fix_spiral_edge_cases,
                           const bool         with_road_signals) :
    xodr_file(xodr_file)
{
    pugi::xml_document           xml_doc;
    const pugi::xml_parse_result xml_parse_result = xml_doc.load_file(xodr_file.c_str());
    if (!xml_parse_result)
        log::error("Error parsing xml: {}", xml_parse_result.description());

    const pugi::xml_node odr_node = xml_doc.child("OpenDRIVE");

    if (const pugi::xml_node geoReference_node = odr_node.child("header").child("geoReference"))
        this->proj4 = geoReference_node.text().as_string("");

    std::size_t cnt_geoms = 1;
    if (center_map)
    {
        for (const pugi::xml_node road_node : odr_node.children("road"))
        {
            for (const pugi::xml_node geometry_hdr_node : road_node.child("planView").children("geometry"))
            {
                const double x0 = geometry_hdr_node.attribute("x").as_double(0.0);
                this->x_offs = this->x_offs + ((x0 - this->x_offs) / cnt_geoms);
                const double y0 = geometry_hdr_node.attribute("y").as_double(0.0);
                this->y_offs = this->y_offs + ((y0 - this->y_offs) / cnt_geoms);
                cnt_geoms++;
            }
        }
    }

    // Roads
    odr::check(odr_node.child("road"), "No roads found");
    for (const pugi::xml_node road_node : odr_node.children("road"))
    {
        const std::string road_id = road_node.attribute("id").as_string("");
        if (this->id_to_road.find(road_id) != this->id_to_road.end())
        {
            log::error("{}: duplicate id {}", node_path(road_node), road_id);
            continue;
        }

        const double length = road_node.attribute("length").as_double(NAN);
        if (std::isnan(length) || length < 0)
        {
            log::error("{}: length {} < 0", node_path(road_node), length);
            continue;
        }

        std::string rule_str = std::string(road_node.attribute("rule").as_string("RHT"));
        std::transform(rule_str.begin(), rule_str.end(), rule_str.begin(), [](unsigned char c) { return std::tolower(c); });
        const bool is_left_hand_traffic = (rule_str == "lht");

        Road road(road_id, length, road_node.attribute("junction").as_string(""), road_node.attribute("name").as_string(""), is_left_hand_traffic);

        // parse road links
        for (const bool is_predecessor : {true, false})
        {
            const pugi::xml_node road_link_node =
                is_predecessor ? road_node.child("link").child("predecessor") : road_node.child("link").child("successor");
            if (road_link_node)
            {
                RoadLink& link = is_predecessor ? road.predecessor : road.successor;
                link.id = road_link_node.attribute("elementId").as_string("");

                const std::string type_str = road_link_node.attribute("elementType").as_string("");
                if (!(type_str == "road" || type_str == "junction"))
                {
                    log::warn("{}: unknown elementType '{}'", node_path(road_link_node), type_str);
                }
                else
                {
                    link.type = (type_str == "road") ? RoadLink::Type::Road : RoadLink::Type::Junction;
                }

                if (link.type == RoadLink::Type::Road)
                {
                    // junction connection has no contact point
                    const std::string contact_point_str = road_link_node.attribute("contactPoint").as_string("");
                    if (!(contact_point_str == "start" || contact_point_str == "end"))
                    {
                        log::warn("{}: unknown contactPoint '{}'", node_path(road_link_node), contact_point_str);
                    }
                    else
                    {
                        link.contact_point = (contact_point_str == "start") ? RoadLink::ContactPoint::Start : RoadLink::ContactPoint::End;
                    }
                }
            }
        }

        // parse road neighbors
        for (const pugi::xml_node road_neighbor_node : road_node.child("link").children("neighbor"))
        {
            const std::string road_neighbor_id = road_neighbor_node.attribute("elementId").as_string("");
            const std::string road_neighbor_side = road_neighbor_node.attribute("side").as_string("");
            const std::string road_neighbor_direction = road_neighbor_node.attribute("direction").as_string("");
            road.neighbors.emplace_back(road_neighbor_id, road_neighbor_side, road_neighbor_direction);
        }

        // parse road type and speed
        for (const pugi::xml_node road_type_node : road_node.children("type"))
        {
            const double      s = road_type_node.attribute("s").as_double(NAN);
            const std::string type = road_type_node.attribute("type").as_string("");
            if (std::isnan(s) || s < 0)
            {
                log::warn("{}: s < 0", node_path(road_type_node));
                continue;
            }
            road.s_to_type[s] = type;

            if (const pugi::xml_node node = road_type_node.child("speed"))
            {
                const std::string speed_record_max = node.attribute("max").as_string("");
                const std::string speed_record_unit = node.attribute("unit").as_string("");
                road.s_to_speed.emplace(s, SpeedRecord(speed_record_max, speed_record_unit));
            }
        }

        // make ref_line - parse road geometries
        bool invalid_geometry = false;
        for (const pugi::xml_node geometry_hdr_node : road_node.child("planView").children("geometry"))
        {
            const double s0 = geometry_hdr_node.attribute("s").as_double(NAN);
            const double hdg0 = geometry_hdr_node.attribute("hdg").as_double(NAN);
            const double length = geometry_hdr_node.attribute("length").as_double(NAN);
            double       x0 = geometry_hdr_node.attribute("x").as_double(NAN);
            double       y0 = geometry_hdr_node.attribute("y").as_double(NAN);

            if (std::isnan(s0) || s0 < 0)
            {
                log::error("{} s < 0", node_path(geometry_hdr_node));
                invalid_geometry = true;
                continue;
            }
            if (std::isnan(x0) || std::isnan(y0) || std::isnan(hdg0))
            {
                log::error("{}: invalid values; x={}, y={}, hdg={}", node_path(geometry_hdr_node), x0, y0, hdg0);
                invalid_geometry = true;
                continue;
            }
            if (std::isnan(length) || length < 0)
            {
                log::error("{}: length {} < 0", node_path(geometry_hdr_node), length);
                invalid_geometry = true;
                continue;
            }

            x0 -= this->x_offs;
            y0 -= this->y_offs;

            const pugi::xml_node geometry_node = geometry_hdr_node.first_child();
            const std::string    geometry_type = geometry_node.name();
            if (geometry_type == "line")
            {
                road.ref_line.s0_to_geometry[s0] = std::make_unique<Line>(s0, x0, y0, hdg0, length);
            }
            else if (geometry_type == "spiral")
            {
                const double curv_start = geometry_node.attribute("curvStart").as_double(NAN);
                const double curv_end = geometry_node.attribute("curvEnd").as_double(NAN);
                if (std::isnan(curv_start) || std::isnan(curv_end))
                {
                    log::error("{}: invalid values; curvStart={}, curvEnd={}", node_path(geometry_node), curv_start, curv_end);
                    invalid_geometry = true;
                    continue;
                }
                if (!fix_spiral_edge_cases)
                {
                    road.ref_line.s0_to_geometry[s0] = std::make_unique<Spiral>(s0, x0, y0, hdg0, length, curv_start, curv_end);
                }
                else
                {
                    if (std::abs(curv_start) < 1e-6 && std::abs(curv_end) < 1e-6)
                    {
                        // In effect a line
                        road.ref_line.s0_to_geometry[s0] = std::make_unique<Line>(s0, x0, y0, hdg0, length);
                    }
                    else if (std::abs(curv_end - curv_start) < 1e-6)
                    {
                        // In effect an arc
                        road.ref_line.s0_to_geometry[s0] = std::make_unique<Arc>(s0, x0, y0, hdg0, length, curv_start);
                    }
                    else
                    {
                        // True spiral
                        road.ref_line.s0_to_geometry[s0] = std::make_unique<Spiral>(s0, x0, y0, hdg0, length, curv_start, curv_end);
                    }
                }
            }
            else if (geometry_type == "arc")
            {
                const double curvature = geometry_node.attribute("curvature").as_double(NAN);
                if (std::isnan(curvature))
                {
                    log::error("{}: invalid curvature", node_path(geometry_node));
                    invalid_geometry = true;
                    continue;
                }
                road.ref_line.s0_to_geometry[s0] = std::make_unique<Arc>(s0, x0, y0, hdg0, length, curvature);
            }
            else if (geometry_type == "paramPoly3")
            {
                const double aU = geometry_node.attribute("aU").as_double(NAN);
                const double bU = geometry_node.attribute("bU").as_double(NAN);
                const double cU = geometry_node.attribute("cU").as_double(NAN);
                const double dU = geometry_node.attribute("dU").as_double(NAN);
                const double aV = geometry_node.attribute("aV").as_double(NAN);
                const double bV = geometry_node.attribute("bV").as_double(NAN);
                const double cV = geometry_node.attribute("cV").as_double(NAN);
                const double dV = geometry_node.attribute("dV").as_double(NAN);
                if (std::isnan(aU) || std::isnan(bU) || std::isnan(cU) || std::isnan(dU) || std::isnan(aV) || std::isnan(bV) || std::isnan(cV) ||
                    std::isnan(dV))
                {
                    log::error("{}: invalid values; aU={}, bU={}, cU={}, dU={}, aV={}, bV={}, cV={}, dV={}",
                               node_path(geometry_node),
                               aU,
                               bU,
                               cU,
                               dU,
                               aV,
                               bV,
                               cV,
                               dV);
                    invalid_geometry = true;
                    continue;
                }

                bool pRange_normalized = true;
                if (geometry_node.attribute("pRange") || geometry_hdr_node.attribute("pRange"))
                {
                    std::string pRange_str = geometry_node.attribute("pRange") ? geometry_node.attribute("pRange").as_string("")
                                                                               : geometry_hdr_node.attribute("pRange").as_string("");
                    std::transform(pRange_str.begin(), pRange_str.end(), pRange_str.begin(), [](unsigned char c) { return std::tolower(c); });
                    if (pRange_str == "arclength")
                        pRange_normalized = false;
                }
                road.ref_line.s0_to_geometry[s0] =
                    std::make_unique<ParamPoly3>(s0, x0, y0, hdg0, length, aU, bU, cU, dU, aV, bV, cV, dV, pRange_normalized);
            }
            else
            {
                log::error("{}: unknown geometry", node_path(geometry_node));
                invalid_geometry = true;
                continue;
            }
        }
        if (invalid_geometry)
        {
            log::error("{}: invalid geometry", node_path(road_node));
            continue; // discard road
        }

        std::map<std::string /*x path query*/, CubicProfile&> cubic_profile_fields{
            {".//elevationProfile//elevation", road.ref_line.elevation_profile}, {".//lanes//laneOffset", road.lane_offset}};
        if (with_lateral_profile)
            cubic_profile_fields.insert({".//lateralProfile//superelevation", road.superelevation});

        // parse elevation profiles, lane offsets, superelevation
        bool invalid_cubic = false;
        for (auto& [xpath_query_str, cubic_profile] : cubic_profile_fields)
        {
            pugi::xpath_node_set xnodes = road_node.select_nodes(xpath_query_str.c_str());
            for (pugi::xpath_node xnode : xnodes)
            {
                const pugi::xml_node node = xnode.node();

                const double s0 = node.attribute("s").as_double(NAN);
                const double a = node.attribute("a").as_double(NAN);
                const double b = node.attribute("b").as_double(NAN);
                const double c = node.attribute("c").as_double(NAN);
                const double d = node.attribute("d").as_double(NAN);

                if (std::isnan(s0) || s0 < 0)
                {
                    log::error("{}: s < 0", node_path(node));
                    invalid_cubic = true;
                    continue;
                }
                if (std::isnan(a) || std::isnan(b) || std::isnan(c) || std::isnan(d))
                {
                    log::error("{}: invalid values; a={}, b={}, c={}, d={}", node_path(node), a, b, c, d);
                    invalid_cubic = true;
                    continue;
                }

                cubic_profile.segments.emplace(s0, CubicPoly(a, b, c, d, s0));
            }
        }
        if (invalid_cubic)
        {
            log::error("{}: has an invalid cubic profile", node_path(road_node));
            continue; // discard road
        }

        // parse crossfall - has extra attribute side
        if (with_lateral_profile)
        {
            for (const pugi::xml_node crossfall_node : road_node.child("lateralProfile").children("crossfall"))
            {
                const double s0 = crossfall_node.attribute("s").as_double(NAN);
                const double a = crossfall_node.attribute("a").as_double(NAN);
                const double b = crossfall_node.attribute("b").as_double(NAN);
                const double c = crossfall_node.attribute("c").as_double(NAN);
                const double d = crossfall_node.attribute("d").as_double(NAN);

                if (std::isnan(s0) || s0 < 0)
                {
                    log::warn("{}: s < 0", node_path(crossfall_node));
                    continue;
                }
                if (std::isnan(a) || std::isnan(b) || std::isnan(c) || std::isnan(d))
                {
                    log::warn("{}: invalid values; a={}, b={}, c={}, d={}", node_path(crossfall_node), a, b, c, d);
                    continue;
                }

                road.crossfall.segments.emplace(s0, CubicPoly(a, b, c, d, s0));
                if (const pugi::xml_attribute side = crossfall_node.attribute("side"))
                {
                    std::string side_str = side.as_string("");
                    std::transform(side_str.begin(), side_str.end(), side_str.begin(), [](unsigned char c) { return std::tolower(c); });
                    if (side_str == "left")
                        road.crossfall.s_to_side[s0] = Crossfall::Side::Left;
                    else if (side_str == "right")
                        road.crossfall.s_to_side[s0] = Crossfall::Side::Right;
                    else // default to 'both'
                        road.crossfall.s_to_side[s0] = Crossfall::Side::Both;
                }
            }

            // check for lateralProfile shape - not implemented yet
            if (road_node.child("lateralProfile").child("shape"))
            {
                log::warn("{}: lateralProfile::shape not supported", node_path(road_node));
            }
        }

        // parse road lane sections and lanes
        bool invalid_lanesection = false;
        for (const pugi::xml_node lanesection_node : road_node.child("lanes").children("laneSection"))
        {
            const double s0 = lanesection_node.attribute("s").as_double(NAN);
            if (std::isnan(s0) || s0 < 0)
            {
                log::error("{}: s < 0", node_path(lanesection_node));
                invalid_lanesection = true;
                continue;
            }

            LaneSection& lanesection = road.s_to_lanesection.emplace(s0, LaneSection(road_id, s0)).first->second;

            for (const pugi::xpath_node lane_xpath_node : lanesection_node.select_nodes(".//lane"))
            {
                const pugi::xml_node lane_node = lane_xpath_node.node();
                const int            lane_id = lane_node.attribute("id").as_int(0);

                odr::check(!lane_node.child("border"), "Road #{} LaneSection {} Lane #{}: border definitions not supported", road_id, s0, lane_id);

                Lane& lane =
                    lanesection.id_to_lane
                        .emplace(lane_id,
                                 Lane(road_id, s0, lane_id, lane_node.attribute("level").as_bool(false), lane_node.attribute("type").as_string("")))
                        .first->second;

                if (const pugi::xml_attribute id_attr = lane_node.child("link").child("predecessor").attribute("id"))
                    lane.predecessor = id_attr.as_int();
                if (const pugi::xml_attribute id_attr = lane_node.child("link").child("successor").attribute("id"))
                    lane.successor = id_attr.as_int();

                for (const pugi::xml_node lane_width_node : lane_node.children("width"))
                {
                    const double s_offset = lane_width_node.attribute("sOffset").as_double(NAN);
                    const double a = lane_width_node.attribute("a").as_double(NAN);
                    const double b = lane_width_node.attribute("b").as_double(NAN);
                    const double c = lane_width_node.attribute("c").as_double(NAN);
                    const double d = lane_width_node.attribute("d").as_double(NAN);

                    if (std::isnan(s_offset) || s_offset < 0)
                    {
                        log::error("{}: sOffset {} < 0", node_path(lane_width_node), s_offset);
                        invalid_lanesection = true;
                        continue;
                    }
                    if (std::isnan(a) || std::isnan(b) || std::isnan(c) || std::isnan(d))
                    {
                        log::error("{}: invalid values; sOffset={}, a={}, b={}, c={}, d={}", node_path(lane_width_node), s_offset, a, b, c, d);
                        invalid_lanesection = true;
                        continue;
                    }

                    CubicPoly width_poly(a, b, c, d, s0 + s_offset);

                    // OpenDRIVE Format Specification, Rev. 1.4, 3.3.1 General:
                    // "The reference line itself is defined as lane zero and must not have a width entry (i.e. its width must always be 0.0)."
                    if (lane_id == 0 && !width_poly.is_zero())
                    {
                        log::warn("{}: width must be 0 for lane #0, setting to 0", node_path(lane_width_node));
                        width_poly.set_zero();
                    }

                    lane.lane_width.segments.emplace(s0 + s_offset, width_poly);
                }

                if (with_lane_height)
                {
                    for (const pugi::xml_node lane_height_node : lane_node.children("height"))
                    {
                        const double s_offset = lane_height_node.attribute("sOffset").as_double(NAN);
                        const double inner = lane_height_node.attribute("inner").as_double(NAN);
                        const double outer = lane_height_node.attribute("outer").as_double(NAN);

                        if (std::isnan(s_offset) || s_offset < 0)
                        {
                            log::warn("{}: sOffset {} < 0", node_path(lane_height_node), s_offset);
                            continue;
                        }
                        if (std::isnan(inner) || std::isnan(outer))
                        {
                            log::warn("{}: invalid values; sOffset={}, inner={}, outer={}", node_path(lane_height_node), s_offset, inner, outer);
                            continue;
                        }

                        lane.s_to_height_offset.emplace(s0 + s_offset, HeightOffset(inner, outer));
                    }
                }

                for (const pugi::xml_node roadmark_node : lane_node.children("roadMark"))
                {
                    const double s_offset = roadmark_node.attribute("sOffset").as_double(NAN);
                    if (std::isnan(s_offset) || s_offset < 0)
                    {
                        log::warn("{}: sOffset {} < 0", node_path(roadmark_node), s_offset);
                        continue;
                    }

                    RoadMarkGroup roadmark_group(road_id,
                                                 s0,
                                                 lane_id,
                                                 roadmark_node.attribute("width").as_double(-1), // optional
                                                 roadmark_node.attribute("height").as_double(0),
                                                 s_offset,
                                                 roadmark_node.attribute("type").as_string("none"),
                                                 roadmark_node.attribute("weight").as_string("standard"),
                                                 roadmark_node.attribute("color").as_string("standard"),
                                                 roadmark_node.attribute("material").as_string("standard"),
                                                 roadmark_node.attribute("laneChange").as_string("both"));
                    const double  roadmark_group_s0 = s0 + roadmark_group.s_offset;

                    if (const pugi::xml_node roadmark_type_node = roadmark_node.child("type"))
                    {
                        const std::string name = roadmark_type_node.attribute("name").as_string("");
                        const double      line_width_1 = roadmark_type_node.attribute("width").as_double(-1);

                        for (const pugi::xml_node roadmarks_line_node : roadmark_type_node.children("line"))
                        {
                            const double line_width_0 = roadmarks_line_node.attribute("width").as_double(NAN);
                            const double roadmark_width = std::isnan(line_width_0) ? line_width_1 : line_width_0;

                            const double roadmark_s_offset = roadmarks_line_node.attribute("sOffset").as_double(NAN);
                            const double roadmark_length = roadmarks_line_node.attribute("length").as_double(NAN);
                            const double roadmark_space = roadmarks_line_node.attribute("space").as_double(NAN);

                            bool values_ok = true;
                            for (auto [val_name, val_ptr] : {std::pair{"sOffset", &roadmark_s_offset},
                                                             std::pair{"length", &roadmark_length},
                                                             std::pair{"space", &roadmark_space}})
                            {
                                if (std::isnan(*val_ptr) || (*val_ptr) < 0)
                                {
                                    log::warn("{}: {} {} < 0", node_path(roadmarks_line_node), val_name, *val_ptr);
                                    values_ok = false;
                                }
                            }
                            if (!values_ok)
                                continue;

                            const double roadmark_t_offset = roadmarks_line_node.attribute("tOffset").as_double(NAN);
                            if (std::isnan(roadmark_t_offset))
                            {
                                log::warn("{}: tOffset NAN", node_path(roadmark_type_node));
                                continue;
                            }

                            roadmark_group.roadmark_lines.emplace(road_id,
                                                                  s0,
                                                                  lane_id,
                                                                  roadmark_group_s0,
                                                                  roadmark_width,
                                                                  roadmark_length,
                                                                  roadmark_space,
                                                                  roadmark_t_offset,
                                                                  roadmark_s_offset,
                                                                  name,
                                                                  roadmarks_line_node.attribute("rule").as_string("none"));
                        }
                    }

                    lane.roadmark_groups.emplace(std::move(roadmark_group));
                }
            }

            // derive lane borders from lane widths
            const auto id_lane_iter0 = lanesection.id_to_lane.find(0);
            if (id_lane_iter0 == lanesection.id_to_lane.end())
                throw std::runtime_error("lane section does not have lane #0");

            // iterate from lane #1 towards +inf
            const auto id_lane_iter1 = std::next(id_lane_iter0);
            for (auto iter = id_lane_iter1; iter != lanesection.id_to_lane.end(); iter++)
            {
                if (iter == id_lane_iter1)
                    iter->second.outer_border = iter->second.lane_width;
                else
                    iter->second.outer_border = std::prev(iter)->second.outer_border.add(iter->second.lane_width);
            }

            // iterate from lane #-1 towards -inf
            // "For a reverse iterator r constructed from an iterator i, the relationship &*r == &*(i - 1) is always true"
            // The reverse iterator points to the element that is one before the element referred by the id_lane_iter0!
            const std::map<int, Lane>::reverse_iterator r_id_lane_iter1(id_lane_iter0);
            for (auto r_iter = r_id_lane_iter1; r_iter != lanesection.id_to_lane.rend(); r_iter++)
            {
                if (r_iter == r_id_lane_iter1)
                    r_iter->second.outer_border = r_iter->second.lane_width.negate();
                else
                    r_iter->second.outer_border = std::prev(r_iter)->second.outer_border.add(r_iter->second.lane_width.negate());
            }

            // OpenDRIVE Format Specification, Rev. 1.4, 3.3.2 Lane Offset:
            // "... lane 0 may be offset using a cubic polynom"
            for (auto& id_lane : lanesection.id_to_lane)
            {
                id_lane.second.outer_border = id_lane.second.outer_border.add(road.lane_offset);
            }
        }
        if (invalid_lanesection)
        {
            log::error("{}: invalid LaneSection", node_path(road_node));
            continue; // discard road
        }

        // parse road objects
        if (with_road_objects)
        {
            const RoadObjectCorner::Type default_local_outline_type =
                abs_z_for_for_local_road_obj_outline ? RoadObjectCorner::Type::Local_AbsZ : RoadObjectCorner::Type::Local_RelZ;

            for (const pugi::xml_node object_node : road_node.child("objects").children("object"))
            {
                const std::string object_id = object_node.attribute("id").as_string("");
                if (road.id_to_object.find(object_id) != road.id_to_object.end())
                {
                    log::warn("{}: duplicate Object #{}", node_path(object_node), object_id);
                    continue;
                }

                std::optional<RoadObject> road_object;
                try
                {
                    road_object.emplace(road_id,
                                        object_id,
                                        object_node.attribute("s").as_double(NAN),
                                        object_node.attribute("t").as_double(NAN),
                                        object_node.attribute("zOffset").as_double(NAN),
                                        try_get_attribute<double>(object_node, "length"),
                                        try_get_attribute<double>(object_node, "validLength"),
                                        try_get_attribute<double>(object_node, "width"),
                                        try_get_attribute<double>(object_node, "radius"),
                                        try_get_attribute<double>(object_node, "height"),
                                        try_get_attribute<double>(object_node, "hdg"),
                                        try_get_attribute<double>(object_node, "pitch"),
                                        try_get_attribute<double>(object_node, "roll"),
                                        try_get_attribute<std::string>(object_node, "type"),
                                        try_get_attribute<std::string>(object_node, "name"),
                                        try_get_attribute<std::string>(object_node, "orientation"),
                                        try_get_attribute<std::string>(object_node, "subtype"),
                                        try_get_attribute<bool>(object_node, "dynamic"));
                }
                catch (const std::exception& ex)
                {
                    log::warn("{}: {}", node_path(object_node), ex.what());
                    continue;
                }

                for (const pugi::xml_node repeat_node : object_node.children("repeat"))
                {
                    try
                    {
                        road_object->repeats.emplace_back(repeat_node.attribute("s").as_double(NAN),
                                                          repeat_node.attribute("length").as_double(NAN),
                                                          repeat_node.attribute("distance").as_double(NAN),
                                                          repeat_node.attribute("tStart").as_double(NAN),
                                                          repeat_node.attribute("tEnd").as_double(NAN),
                                                          repeat_node.attribute("heightStart").as_double(NAN),
                                                          repeat_node.attribute("heightEnd").as_double(NAN),
                                                          repeat_node.attribute("zOffsetStart").as_double(NAN),
                                                          repeat_node.attribute("zOffsetEnd").as_double(NAN),
                                                          try_get_attribute<double>(repeat_node, "widthStart"),
                                                          try_get_attribute<double>(repeat_node, "widthEnd"));
                    }
                    catch (const std::exception& ex)
                    {
                        log::warn("{}: {}", node_path(repeat_node), ex.what());
                    }
                }

                // since v1.45 multiple <outline> are allowed and parent tag is <outlines>, not <object>; this supports v1.4 and v1.45+
                const pugi::xml_node outlines_parent_node = object_node.child("outlines") ? object_node.child("outlines") : object_node;
                for (const pugi::xml_node outline_node : outlines_parent_node.children("outline"))
                {
                    RoadObjectOutline road_object_outline(try_get_attribute<int>(outline_node, "id"),
                                                          try_get_attribute<std::string>(outline_node, "fillType"),
                                                          try_get_attribute<std::string>(outline_node, "laneType"),
                                                          try_get_attribute<bool>(outline_node, "outer"),
                                                          try_get_attribute<bool>(outline_node, "closed"));

                    for (const pugi::xml_node corner_local_node : outline_node.children("cornerLocal"))
                    {
                        const Vec3D pt_local{corner_local_node.attribute("u").as_double(NAN),
                                             corner_local_node.attribute("v").as_double(NAN),
                                             corner_local_node.attribute("z").as_double(NAN)};
                        try
                        {
                            road_object_outline.outline.emplace_back(pt_local,
                                                                     corner_local_node.attribute("height").as_double(NAN),
                                                                     default_local_outline_type,
                                                                     try_get_attribute<int>(corner_local_node, "id"));
                        }
                        catch (const std::exception& ex)
                        {
                            log::warn("{}: {}", node_path(corner_local_node), ex.what());
                        }
                    }

                    for (const pugi::xml_node corner_road_node : outline_node.children("cornerRoad"))
                    {
                        const Vec3D pt_road{corner_road_node.attribute("s").as_double(NAN),
                                            corner_road_node.attribute("t").as_double(NAN),
                                            corner_road_node.attribute("dz").as_double(NAN)};
                        try
                        {
                            road_object_outline.outline.emplace_back(pt_road,
                                                                     corner_road_node.attribute("height").as_double(NAN),
                                                                     RoadObjectCorner::Type::Road,
                                                                     try_get_attribute<int>(corner_road_node, "id"));
                        }
                        catch (const std::exception& ex)
                        {
                            log::warn("{}: {}", node_path(corner_road_node), ex.what());
                        }
                    }

                    road_object->outlines.push_back(std::move(road_object_outline));
                }

                for (const pugi::xml_node validity_node : object_node.children("validity"))
                {
                    if (!(validity_node.attribute("fromLane") && validity_node.attribute("toLane")))
                    {
                        log::warn("{}: 'fromLane' or 'toLane' missing", node_path(validity_node));
                        continue;
                    }
                    const int from_lane = validity_node.attribute("fromLane").as_int(INT_MIN);
                    const int to_lane = validity_node.attribute("toLane").as_int(INT_MAX);
                    road_object->lane_validities.emplace_back(from_lane, to_lane);
                }

                road.id_to_object.emplace(object_id, std::move(*road_object));
            }
        }
        // parse signals
        if (with_road_signals)
        {
            for (const pugi::xml_node signal_node : road_node.child("signals").children("signal"))
            {
                const std::string signal_id = signal_node.attribute("id").as_string("");
                if (road.id_to_signal.find(signal_id) != road.id_to_signal.end())
                {
                    log::warn("{}: duplicate Signal #{}", node_path(signal_node), signal_id);
                    continue;
                }

                std::optional<RoadSignal> road_signal;
                try
                {
                    road_signal.emplace(signal_id,
                                        road_id,
                                        signal_node.attribute("s").as_double(NAN),
                                        signal_node.attribute("t").as_double(NAN),
                                        signal_node.attribute("zOffset").as_double(NAN),
                                        signal_node.attribute("dynamic").as_bool(),
                                        signal_node.attribute("type").as_string("none"),
                                        signal_node.attribute("subtype").as_string("none"),
                                        signal_node.attribute("orientation").as_string("none"),
                                        try_get_attribute<double>(signal_node, "value"),
                                        try_get_attribute<double>(signal_node, "height"),
                                        try_get_attribute<double>(signal_node, "width"),
                                        try_get_attribute<double>(signal_node, "hOffset"),
                                        try_get_attribute<double>(signal_node, "pitch"),
                                        try_get_attribute<double>(signal_node, "roll"),
                                        try_get_attribute<std::string>(signal_node, "name"),
                                        try_get_attribute<std::string>(signal_node, "unit"),
                                        try_get_attribute<std::string>(signal_node, "text"),
                                        try_get_attribute<std::string>(signal_node, "country"));
                }
                catch (const std::exception& ex)
                {
                    log::warn("{}: {}", node_path(signal_node), ex.what());
                    continue;
                }

                for (const pugi::xml_node validity_node : signal_node.children("validity"))
                {
                    if (!(validity_node.attribute("fromLane") && validity_node.attribute("toLane")))
                    {
                        log::warn("{}: 'fromLane' or 'toLane' missing", node_path(validity_node));
                        continue;
                    }
                    const int from_lane = validity_node.attribute("fromLane").as_int(INT_MIN);
                    const int to_lane = validity_node.attribute("toLane").as_int(INT_MAX);
                    road_signal->lane_validities.emplace_back(from_lane, to_lane);
                }

                road.id_to_signal.emplace(signal_id, std::move(*road_signal));
            }
        }

        this->id_to_road.emplace(road.id, std::move(road));
    }

    // Junctions
    for (const pugi::xml_node junction_node : odr_node.children("junction"))
    {
        const std::string id = junction_node.attribute("id").as_string("");
        if (this->id_to_junction.find(id) != this->id_to_junction.end())
        {
            log::error("duplicate Junction #{}", id);
            continue;
        }

        Junction& junction = this->id_to_junction.emplace(id, Junction(junction_node.attribute("name").as_string(""), id)).first->second;

        for (const pugi::xml_node connection_node : junction_node.children("connection"))
        {
            const std::string conn_id = connection_node.attribute("id").as_string("");
            if (junction.id_to_connection.find(conn_id) != junction.id_to_connection.end())
            {
                log::warn("{}: duplicate Connection #{}", node_path(connection_node), conn_id);
                continue;
            }

            const std::string contact_point_str = connection_node.attribute("contactPoint").as_string("");
            if (!(contact_point_str == "start" || contact_point_str == "end"))
            {
                log::warn("{}: unknown contactPoint '{}'", node_path(connection_node), contact_point_str);
                continue;
            }
            const JunctionConnection::ContactPoint contact_point =
                (contact_point_str == "start") ? JunctionConnection::ContactPoint::Start : JunctionConnection::ContactPoint::End;

            const std::string road_in = connection_node.attribute("incomingRoad").as_string("");
            const std::string road_conn = connection_node.attribute("connectingRoad").as_string("");

            JunctionConnection& connection =
                junction.id_to_connection.emplace(conn_id, JunctionConnection(conn_id, road_in, road_conn, contact_point)).first->second;

            for (const pugi::xml_node lane_link_node : connection_node.children("laneLink"))
            {
                const int from_lane = lane_link_node.attribute("from").as_int(0);
                const int to_lane = lane_link_node.attribute("to").as_int(0);
                connection.lane_links.emplace(from_lane, to_lane);
            }
        }

        const std::size_t num_conns = junction.id_to_connection.size();
        if (num_conns == 0)
            log::warn("{}: 0 connections", node_path(junction_node));

        for (const pugi::xml_node priority_node : junction_node.children("priority"))
        {
            const std::string prio_high = priority_node.attribute("high").as_string("");
            const std::string prio_low = priority_node.attribute("low").as_string("");
            if (prio_low.empty() || prio_high.empty())
            {
                log::warn("{}: empty priority; high='{}', low='{}'", node_path(priority_node), prio_high, prio_low);
                continue;
            }
            junction.priorities.emplace(prio_high, prio_low);
        }

        for (const pugi::xml_node controller_node : junction_node.children("controller"))
        {
            const std::string controller_id = controller_node.attribute("id").as_string("");
            if (junction.id_to_controller.find(controller_id) != junction.id_to_controller.end())
            {
                log::warn("{}: duplicate Controller #{}", node_path(controller_node), controller_id);
                continue;
            }
            const int64_t seq_id = controller_node.attribute("sequence").as_llong(-1); // type: uint32_t
            if (seq_id < 0)
            {
                log::warn("{}: sequence {} < 0", node_path(controller_node), seq_id);
                continue;
            }
            const std::string type = controller_node.attribute("type").as_string("");
            junction.id_to_controller.emplace(controller_id, JunctionController(controller_id, type, seq_id));
        }
    }
}

Road OpenDriveMap::get_road(const std::string& id) const
{
    return this->id_to_road.at(id);
}

Junction OpenDriveMap::get_junction(const std::string& id) const
{
    return this->id_to_junction.at(id);
}

std::vector<Road> OpenDriveMap::get_roads() const
{
    return get_map_values(this->id_to_road);
}

std::vector<Junction> OpenDriveMap::get_junctions() const
{
    return get_map_values(this->id_to_junction);
}

const LaneSection* OpenDriveMap::get_adjacent_lanesection(const std::string& road_id, const double& lanesection_s0, const bool predecessor) const
{
    const Road& road = this->id_to_road.at(road_id);
    const auto  s_lanesec_iter = road.s_to_lanesection.find(lanesection_s0);
    if (s_lanesec_iter == road.s_to_lanesection.end()) // also catches empty road
        return nullptr;

    if (predecessor)
    {
        if (s_lanesec_iter != road.s_to_lanesection.begin())
            return &(std::prev(s_lanesec_iter)->second);
    }
    else
    {
        const auto next_lanesec_iter = std::next(s_lanesec_iter);
        if (next_lanesec_iter != road.s_to_lanesection.end())
            return &(next_lanesec_iter->second);
    }

    // adjacent lanesection not in the same road
    const RoadLink& road_link = predecessor ? road.predecessor : road.successor;
    if (road_link.type == RoadLink::Type::Road && road_link.contact_point != RoadLink::ContactPoint::None)
    {
        const auto next_road_iter = this->id_to_road.find(road_link.id);
        if (next_road_iter == this->id_to_road.end())
            return nullptr;

        const Road& next_road = next_road_iter->second;
        if (next_road.s_to_lanesection.empty())
            return nullptr;

        const LaneSection& adjacent_lanesection = (road_link.contact_point == RoadLink::ContactPoint::Start)
                                                      ? next_road.s_to_lanesection.begin()->second
                                                      : next_road.s_to_lanesection.rbegin()->second;
        return &adjacent_lanesection;
    }

    return nullptr;
}

const Lane* OpenDriveMap::get_adjacent_lane(const Lane& lane, const bool predecessor) const
{
    const LaneSection* adjacent_lanesec = this->get_adjacent_lanesection(lane.key.road_id, lane.key.lanesection_s0, predecessor);
    if (!adjacent_lanesec)
        return nullptr;

    const std::optional<int> adjacent_lane_id = predecessor ? lane.predecessor : lane.successor;
    if (adjacent_lane_id)
    {
        const auto adjacent_lane_iter = adjacent_lanesec->id_to_lane.find(adjacent_lane_id.value());
        if (adjacent_lane_iter != adjacent_lanesec->id_to_lane.end())
            return &(adjacent_lane_iter->second);
    }

    return nullptr;
}

RoadNetworkMesh OpenDriveMap::get_road_network_mesh(const double eps) const
{
    RoadNetworkMesh  out_mesh;
    LanesMesh&       lanes_mesh = out_mesh.lanes_mesh;
    RoadmarksMesh&   roadmarks_mesh = out_mesh.roadmarks_mesh;
    RoadObjectsMesh& road_objects_mesh = out_mesh.road_objects_mesh;
    RoadSignalsMesh& road_signals_mesh = out_mesh.road_signals_mesh;

    for (const auto& id_road : this->id_to_road)
    {
        const Road& road = id_road.second;
        lanes_mesh.road_start_indices[lanes_mesh.vertices.size()] = road.id;
        roadmarks_mesh.road_start_indices[roadmarks_mesh.vertices.size()] = road.id;
        road_objects_mesh.road_start_indices[road_objects_mesh.vertices.size()] = road.id;

        for (const auto& s_lanesec : road.s_to_lanesection)
        {
            const LaneSection& lanesec = s_lanesec.second;
            lanes_mesh.lanesec_start_indices[lanes_mesh.vertices.size()] = lanesec.s0;
            roadmarks_mesh.lanesec_start_indices[roadmarks_mesh.vertices.size()] = lanesec.s0;
            for (const auto& id_lane : lanesec.id_to_lane)
            {
                const Lane&       lane = id_lane.second;
                const std::size_t lanes_idx_offset = lanes_mesh.vertices.size();
                lanes_mesh.lane_start_indices[lanes_idx_offset] = lane.id;
                lanes_mesh.add_mesh(road.get_lane_mesh(lane, eps));

                std::size_t roadmarks_idx_offset = roadmarks_mesh.vertices.size();
                roadmarks_mesh.lane_start_indices[roadmarks_idx_offset] = lane.id;
                const std::vector<RoadMark> roadmarks = lane.get_roadmarks(lanesec.s0, road.get_lanesection_end(lanesec));
                for (const RoadMark& roadmark : roadmarks)
                {
                    roadmarks_idx_offset = roadmarks_mesh.vertices.size();
                    roadmarks_mesh.roadmark_type_start_indices[roadmarks_idx_offset] = roadmark.type;
                    roadmarks_mesh.add_mesh(road.get_roadmark_mesh(lane, roadmark, eps));
                }
            }
        }

        for (const auto& id_road_object : road.id_to_object)
        {
            const RoadObject& road_object = id_road_object.second;
            const std::size_t road_objs_idx_offset = road_objects_mesh.vertices.size();
            road_objects_mesh.road_object_start_indices[road_objs_idx_offset] = road_object.id;
            road_objects_mesh.add_mesh(road.get_road_object_mesh(road_object, eps));
        }

        for (const auto& id_signal : road.id_to_signal)
        {
            const RoadSignal& road_signal = id_signal.second;
            const std::size_t signals_idx_offset = road_signals_mesh.vertices.size();
            road_signals_mesh.road_signal_start_indices[signals_idx_offset] = road_signal.id;
            road_signals_mesh.add_mesh(road.get_road_signal_mesh(road_signal));
        }
    }

    return out_mesh;
}

RoutingGraph OpenDriveMap::get_routing_graph() const
{
    RoutingGraph routing_graph;

    // Parse Roads
    for (const auto& [_, road] : id_to_road)
    {
        for (const auto& [_, lanesection] : road.s_to_lanesection)
        {
            for (const auto& [_, lane] : lanesection.id_to_lane)
            {
                const bool lane_follows_road_direction = lane.key.lane_id < 0;

                const Lane* predecessor_lane = this->get_adjacent_lane(lane, lane_follows_road_direction);
                if (predecessor_lane)
                {
                    const Road&  predecessor_road = this->id_to_road.at(predecessor_lane->key.road_id);
                    const double lane_length = predecessor_road.get_lanesection_length(predecessor_lane->key.lanesection_s0);
                    routing_graph.add_edge(RoutingGraphEdge(predecessor_lane->key, lane.key, lane_length));
                }

                const Lane* successor_lane = this->get_adjacent_lane(lane, !lane_follows_road_direction);
                if (successor_lane)
                {
                    const double lane_length = road.get_lanesection_length(lane.key.lanesection_s0);
                    routing_graph.add_edge(RoutingGraphEdge(lane.key, successor_lane->key, lane_length));
                }
            }
        }
    }

    // Parse Junctions
    for (const auto& [_, junction] : id_to_junction)
    {
        for (const auto& [_, conn] : junction.id_to_connection)
        {
            auto incoming_road_iter = id_to_road.find(conn.incoming_road);
            auto connecting_road_iter = id_to_road.find(conn.connecting_road);
            if (incoming_road_iter == id_to_road.end() || connecting_road_iter == id_to_road.end())
                continue;

            const Road& incoming_road = incoming_road_iter->second;
            const Road& connecting_road = connecting_road_iter->second;

            const LaneSection& incoming_lanesec =
                (incoming_road.successor.type == RoadLink::Type::Junction && incoming_road.successor.id == junction.id)
                    ? incoming_road.s_to_lanesection.rbegin()->second
                    : incoming_road.s_to_lanesection.begin()->second;

            const LaneSection& connecting_lanesec = (conn.contact_point == JunctionConnection::ContactPoint::Start)
                                                        ? connecting_road.s_to_lanesection.begin()->second
                                                        : connecting_road.s_to_lanesection.rbegin()->second;

            for (const JunctionLaneLink& lane_link : conn.lane_links)
            {
                if (lane_link.from == 0 || lane_link.to == 0)
                    continue;

                auto from_lane_iter = incoming_lanesec.id_to_lane.find(lane_link.from);
                auto to_lane_iter = connecting_lanesec.id_to_lane.find(lane_link.to);

                if (from_lane_iter == incoming_lanesec.id_to_lane.end() || to_lane_iter == connecting_lanesec.id_to_lane.end())
                    continue;

                const LaneKey from(incoming_road.id, incoming_lanesec.s0, from_lane_iter->second.id);
                const LaneKey to(connecting_road.id, connecting_lanesec.s0, to_lane_iter->second.id);
                const double  lane_length = incoming_road.get_lanesection_length(incoming_lanesec);

                routing_graph.add_edge(RoutingGraphEdge(from, to, lane_length));
            }
        }
    }

    return routing_graph;
}

} // namespace odr
