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

std::vector<LaneValidityRecord> extract_lane_validity_records(const pugi::xml_node& xml_node)
{
    std::vector<LaneValidityRecord> lane_validities;
    for (const auto& validity_node : xml_node.children("validity"))
    {
        LaneValidityRecord lane_validity{validity_node.attribute("fromLane").as_int(INT_MIN), validity_node.attribute("toLane").as_int(INT_MAX)};

        // fromLane should not be greater than toLane, since the standard defines them as follows:
        // fromLane - the minimum ID of lanes for which the object is valid
        // toLane - the maximum ID of lanes for which the object is valid
        // If we find such a violation, we set both IDs to 0 which means the
        // object is only applicable to the centerlane.
        odr::check_and_repair(
            lane_validity.from_lane <= lane_validity.to_lane,
            [&]()
            {
                lane_validity.from_lane = 0;
                lane_validity.to_lane = 0;
            },
            "Lane::validity::fromLane %d > Lane::validity::toLane %d, set to 0",
            lane_validity.from_lane,
            lane_validity.to_lane);

        lane_validities.push_back(std::move(lane_validity));
    }
    return lane_validities;
}

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
        log::error("Error parsing xml: %s", xml_parse_result.description());

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
            log::error("Road #%s already exists, skipping...", road_id.c_str());
            continue;
        }

        const double length = road_node.attribute("length").as_double(NAN);
        if (std::isnan(length) || length < 0)
        {
            log::error("Road #%s: length %f, skipping...", road_id.c_str(), length);
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
                    log::error("Road #%s: unknown link::%s::elementType '%s'",
                               road_id.c_str(),
                               is_predecessor ? "predecessor" : "successor",
                               type_str.c_str());
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
                        log::error("Road #%s: unknown link::%s::contactPoint '%s'",
                                   road_id.c_str(),
                                   is_predecessor ? "predecessor" : "successor",
                                   contact_point_str.c_str());
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
            RoadNeighbor      road_neighbor(road_neighbor_id, road_neighbor_side, road_neighbor_direction);
            road.neighbors.push_back(road_neighbor);
        }

        // parse road type and speed
        for (const pugi::xml_node road_type_node : road_node.children("type"))
        {
            const double      s = road_type_node.attribute("s").as_double(NAN);
            const std::string type = road_type_node.attribute("type").as_string("");

            if (std::isnan(s) || s < 0)
            {
                log::warn("Road #%s: discard type record '%s' s %f < 0", road_id.c_str(), type.c_str(), s);
                continue;
            }

            road.s_to_type[s] = type;
            if (const pugi::xml_node node = road_type_node.child("speed"))
            {
                const std::string speed_record_max = node.attribute("max").as_string("");
                const std::string speed_record_unit = node.attribute("unit").as_string("");
                SpeedRecord       speed_record(speed_record_max, speed_record_unit);
                road.s_to_speed.insert({s, speed_record});
            }
        }

        // make ref_line - parse road geometries
        bool broken_geometry = false;
        for (const pugi::xml_node geometry_hdr_node : road_node.child("planView").children("geometry"))
        {
            const double s0 = geometry_hdr_node.attribute("s").as_double(NAN);
            const double hdg0 = geometry_hdr_node.attribute("hdg").as_double(NAN);
            const double length = geometry_hdr_node.attribute("length").as_double(NAN);
            double       x0 = geometry_hdr_node.attribute("x").as_double(NAN);
            double       y0 = geometry_hdr_node.attribute("y").as_double(NAN);

            if (std::isnan(x0) || std::isnan(y0) || std::isnan(hdg0))
            {
                log::error("Road #%s: broken planView::geometry x=%f, y=%f, hdg=%f", road_id.c_str(), x0, y0, hdg0);
                broken_geometry = true;
                break;
            }
            if (std::isnan(s0) || s0 < 0)
            {
                log::error("Road #%s: planView::geometry::s %f < 0", road_id.c_str(), s0);
                broken_geometry = true;
                break;
            }
            if (std::isnan(length) || length < 0)
            {
                log::error("Road #%s: planView::geometry::length %f < 0", road_id.c_str(), length);
                broken_geometry = true;
                break;
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
                const double curv_start = geometry_node.attribute("curvStart").as_double(0.0);
                const double curv_end = geometry_node.attribute("curvEnd").as_double(0.0);
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
                const double curvature = geometry_node.attribute("curvature").as_double(0.0);
                road.ref_line.s0_to_geometry[s0] = std::make_unique<Arc>(s0, x0, y0, hdg0, length, curvature);
            }
            else if (geometry_type == "paramPoly3")
            {
                const double aU = geometry_node.attribute("aU").as_double(0.0);
                const double bU = geometry_node.attribute("bU").as_double(0.0);
                const double cU = geometry_node.attribute("cU").as_double(0.0);
                const double dU = geometry_node.attribute("dU").as_double(0.0);
                const double aV = geometry_node.attribute("aV").as_double(0.0);
                const double bV = geometry_node.attribute("bV").as_double(0.0);
                const double cV = geometry_node.attribute("cV").as_double(0.0);
                const double dV = geometry_node.attribute("dV").as_double(0.0);

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
                log::error("Could not parse %s", geometry_type.c_str());
                continue;
            }
        }
        if (broken_geometry)
            continue; // don't add road

        std::map<std::string /*x path query*/, CubicProfile&> cubic_profile_fields{
            {".//elevationProfile//elevation", road.ref_line.elevation_profile}, {".//lanes//laneOffset", road.lane_offset}};
        if (with_lateral_profile)
            cubic_profile_fields.insert({".//lateralProfile//superelevation", road.superelevation});

        // parse elevation profiles, lane offsets, superelevation
        for (auto& [xpath_query_str, cubic_profile] : cubic_profile_fields)
        {
            pugi::xpath_node_set nodes = road_node.select_nodes(xpath_query_str.c_str());
            for (pugi::xpath_node node : nodes)
            {
                double       s0 = node.node().attribute("s").as_double(0.0);
                const double a = node.node().attribute("a").as_double(0.0);
                const double b = node.node().attribute("b").as_double(0.0);
                const double c = node.node().attribute("c").as_double(0.0);
                const double d = node.node().attribute("d").as_double(0.0);

                odr::check_and_repair(
                    s0 >= 0, [&]() { s0 = 0; }, "Road #%s: %s s %f < 0, set s=0", road_id.c_str(), xpath_query_str.c_str(), s0);

                cubic_profile.segments.emplace(s0, CubicPoly(a, b, c, d, s0));
            }
        }

        // parse crossfall - has extra attribute side
        if (with_lateral_profile)
        {
            for (pugi::xml_node crossfall_node : road_node.child("lateralProfile").children("crossfall"))
            {
                double       s0 = crossfall_node.attribute("s").as_double(0.0);
                const double a = crossfall_node.attribute("a").as_double(0.0);
                const double b = crossfall_node.attribute("b").as_double(0.0);
                const double c = crossfall_node.attribute("c").as_double(0.0);
                const double d = crossfall_node.attribute("d").as_double(0.0);

                odr::check_and_repair(
                    s0 >= 0, [&]() { s0 = 0; }, "Road #%s: lateralProfile::crossfall::s %f < 0, set s=0", road_id.c_str(), s0);

                road.crossfall.segments.emplace(s0, CubicPoly(a, b, c, d, s0));
                if (const pugi::xml_attribute side = crossfall_node.attribute("side"))
                {
                    std::string side_str = side.as_string("");
                    std::transform(side_str.begin(), side_str.end(), side_str.begin(), [](unsigned char c) { return std::tolower(c); });
                    if (side_str == "left")
                        road.crossfall.s_to_side[s0] = Crossfall::Side::Left;
                    else if (side_str == "right")
                        road.crossfall.s_to_side[s0] = Crossfall::Side::Right;
                    else
                        road.crossfall.s_to_side[s0] = Crossfall::Side::Both;
                }
            }

            // check for lateralProfile shape - not implemented yet
            if (road_node.child("lateralProfile").child("shape"))
            {
                log::error("Lateral Profile Shape not supported");
            }
        }

        // parse road lane sections and lanes
        for (const pugi::xml_node lanesection_node : road_node.child("lanes").children("laneSection"))
        {
            const double s0 = lanesection_node.attribute("s").as_double(0.0);
            LaneSection& lanesection = road.s_to_lanesection.insert({s0, LaneSection(road_id, s0)}).first->second;

            for (const pugi::xpath_node lane_xpath_node : lanesection_node.select_nodes(".//lane"))
            {
                const pugi::xml_node lane_node = lane_xpath_node.node();
                const int            lane_id = lane_node.attribute("id").as_int(0);

                odr::check(
                    !lane_node.child("border"), "Road #%s LaneSection %f Lane #%d: border definitions not supported", road_id.c_str(), s0, lane_id);

                Lane& lane =
                    lanesection.id_to_lane
                        .insert({lane_id,
                                 Lane(road_id, s0, lane_id, lane_node.attribute("level").as_bool(false), lane_node.attribute("type").as_string(""))})
                        .first->second;

                if (const pugi::xml_attribute id_attr = lane_node.child("link").child("predecessor").attribute("id"))
                    lane.predecessor = id_attr.as_int();
                if (const pugi::xml_attribute id_attr = lane_node.child("link").child("successor").attribute("id"))
                    lane.successor = id_attr.as_int();

                for (const pugi::xml_node lane_width_node : lane_node.children("width"))
                {
                    double       s_offset = lane_width_node.attribute("sOffset").as_double(0.0);
                    const double a = lane_width_node.attribute("a").as_double(0.0);
                    const double b = lane_width_node.attribute("b").as_double(0.0);
                    const double c = lane_width_node.attribute("c").as_double(0.0);
                    const double d = lane_width_node.attribute("d").as_double(0.0);

                    odr::check_and_repair(
                        s_offset >= 0,
                        [&]() { s_offset = 0; },
                        "Road #%s LaneSection %f Lane #%d: width::sOffset %f < 0, set sOffset=0",
                        road_id.c_str(),
                        s0,
                        lane_id,
                        s_offset);
                    CubicPoly width_poly(a, b, c, d, s0 + s_offset);

                    // OpenDRIVE Format Specification, Rev. 1.4, 3.3.1 General:
                    // "The reference line itself is defined as lane zero and must not have a width entry (i.e. its width must always be 0.0)."
                    if (lane_id == 0)
                        odr::check_and_repair(
                            width_poly.is_zero(),
                            [&]() { width_poly.set_zero(); },
                            "Road #%s LaneSection %f Lane #0: width must be 0, set to 0",
                            road_id.c_str(),
                            s0);

                    lane.lane_width.segments.emplace(s0 + s_offset, width_poly);
                }

                if (with_lane_height)
                {
                    for (const pugi::xml_node lane_height_node : lane_node.children("height"))
                    {
                        double       s_offset = lane_height_node.attribute("sOffset").as_double(0.0);
                        const double inner = lane_height_node.attribute("inner").as_double(0.0);
                        const double outer = lane_height_node.attribute("outer").as_double(0.0);

                        odr::check_and_repair(
                            s_offset >= 0,
                            [&]() { s_offset = 0; },
                            "Road #%s LaneSection %f Lane #%d: height::sOffset %f < 0, set sOffset=0",
                            road_id.c_str(),
                            s0,
                            lane_id,
                            s_offset);
                        lane.s_to_height_offset.insert({s0 + s_offset, HeightOffset(inner, outer)});
                    }
                }

                for (const pugi::xml_node roadmark_node : lane_node.children("roadMark"))
                {
                    RoadMarkGroup roadmark_group(road_id,
                                                 s0,
                                                 lane_id,
                                                 roadmark_node.attribute("width").as_double(-1),
                                                 roadmark_node.attribute("height").as_double(0),
                                                 roadmark_node.attribute("sOffset").as_double(0),
                                                 roadmark_node.attribute("type").as_string("none"),
                                                 roadmark_node.attribute("weight").as_string("standard"),
                                                 roadmark_node.attribute("color").as_string("standard"),
                                                 roadmark_node.attribute("material").as_string("standard"),
                                                 roadmark_node.attribute("laneChange").as_string("both"));

                    odr::check_and_repair(
                        roadmark_group.s_offset >= 0,
                        [&]() { roadmark_group.s_offset = 0; },
                        "Road #%s LaneSection %f Lane #%d: roadMark::sOffset %f < 0, set sOffset=0",
                        road_id.c_str(),
                        s0,
                        lane_id,
                        roadmark_group.s_offset);
                    const double roadmark_group_s0 = s0 + roadmark_group.s_offset;

                    if (const pugi::xml_node roadmark_type_node = roadmark_node.child("type"))
                    {
                        const std::string name = roadmark_type_node.attribute("name").as_string("");
                        const double      line_width_1 = roadmark_type_node.attribute("width").as_double(-1);

                        for (const pugi::xml_node roadmarks_line_node : roadmark_type_node.children("line"))
                        {
                            const double line_width_0 = roadmarks_line_node.attribute("width").as_double(-1);
                            const double roadmark_width = line_width_0 < 0 ? line_width_1 : line_width_0;

                            RoadMarksLine roadmarks_line(road_id,
                                                         s0,
                                                         lane_id,
                                                         roadmark_group_s0,
                                                         roadmark_width,
                                                         roadmarks_line_node.attribute("length").as_double(0),
                                                         roadmarks_line_node.attribute("space").as_double(0),
                                                         roadmarks_line_node.attribute("tOffset").as_double(0),
                                                         roadmarks_line_node.attribute("sOffset").as_double(0),
                                                         name,
                                                         roadmarks_line_node.attribute("rule").as_string("none"));

                            for (auto [val_name, val_ptr] : {std::pair{"length", &(roadmarks_line.length)},
                                                             std::pair{"space", &(roadmarks_line.space)},
                                                             std::pair{"s_offset", &(roadmarks_line.s_offset)}})
                            {
                                odr::check_and_repair((*val_ptr) >= 0,
                                                      [p = val_ptr]() { *p = 0; },
                                                      "Road #%s LaneSection %f Lane #%d RoadMark %f: type::line::%s %f < 0, set %s=0",
                                                      road_id.c_str(),
                                                      s0,
                                                      lane_id,
                                                      roadmark_group_s0,
                                                      val_name,
                                                      *val_ptr,
                                                      val_name);
                            }

                            roadmark_group.roadmark_lines.emplace(std::move(roadmarks_line));
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

            // OpenDRIVE® Format Specification, Rev. 1.4, 3.3.2 Lane Offset:
            // "... lane 0 may be offset using a cubic polynom"
            for (auto& id_lane : lanesection.id_to_lane)
            {
                id_lane.second.outer_border = id_lane.second.outer_border.add(road.lane_offset);
            }
        }

        // parse road objects
        if (with_road_objects)
        {
            const RoadObjectCorner::Type default_local_outline_type =
                abs_z_for_for_local_road_obj_outline ? RoadObjectCorner::Type::Local_AbsZ : RoadObjectCorner::Type::Local_RelZ;

            for (pugi::xml_node object_node : road_node.child("objects").children("object"))
            {
                std::string road_object_id = object_node.attribute("id").as_string("");
                odr::check_and_repair(
                    road.id_to_object.find(road_object_id) == road.id_to_object.end(),
                    [&]() { road_object_id = road_object_id + std::string("_dup"); },
                    "Road #%s Object #%s already exists, adding _dup suffix to id",
                    road_id.c_str(),
                    road_object_id.c_str());

                const bool  is_dynamic_object = std::string(object_node.attribute("dynamic").as_string("no")) == "yes" ? true : false;
                RoadObject& road_object = road.id_to_object
                                              .insert({road_object_id,
                                                       RoadObject(road_id,
                                                                  road_object_id,
                                                                  object_node.attribute("s").as_double(0),
                                                                  object_node.attribute("t").as_double(0),
                                                                  object_node.attribute("zOffset").as_double(0),
                                                                  object_node.attribute("length").as_double(0),
                                                                  object_node.attribute("validLength").as_double(0),
                                                                  object_node.attribute("width").as_double(0),
                                                                  object_node.attribute("radius").as_double(0),
                                                                  object_node.attribute("height").as_double(0),
                                                                  object_node.attribute("hdg").as_double(0),
                                                                  object_node.attribute("pitch").as_double(0),
                                                                  object_node.attribute("roll").as_double(0),
                                                                  object_node.attribute("type").as_string(""),
                                                                  object_node.attribute("name").as_string(""),
                                                                  object_node.attribute("orientation").as_string(""),
                                                                  object_node.attribute("subtype").as_string(""),
                                                                  is_dynamic_object)})
                                              .first->second;

                for (auto [val_name, val_ptr] : {std::pair{"s0", &(road_object.s0)},
                                                 std::pair{"valid_length", &(road_object.valid_length)},
                                                 std::pair{"length", &(road_object.length)},
                                                 std::pair{"width", &(road_object.width)},
                                                 std::pair{"radius", &(road_object.radius)}})
                {
                    odr::check_and_repair((*val_ptr) >= 0,
                                          [p = val_ptr]() { *p = 0; },
                                          "Road #%s: object::%s %f < 0, set %s=0",
                                          road_id.c_str(),
                                          val_name,
                                          *val_ptr,
                                          val_name);
                }

                for (pugi::xml_node repeat_node : object_node.children("repeat"))
                {
                    RoadObjectRepeat road_object_repeat(repeat_node.attribute("s").as_double(NAN),
                                                        repeat_node.attribute("length").as_double(0),
                                                        repeat_node.attribute("distance").as_double(0),
                                                        repeat_node.attribute("tStart").as_double(NAN),
                                                        repeat_node.attribute("tEnd").as_double(NAN),
                                                        repeat_node.attribute("widthStart").as_double(NAN),
                                                        repeat_node.attribute("widthEnd").as_double(NAN),
                                                        repeat_node.attribute("heightStart").as_double(NAN),
                                                        repeat_node.attribute("heightEnd").as_double(NAN),
                                                        repeat_node.attribute("zOffsetStart").as_double(NAN),
                                                        repeat_node.attribute("zOffsetEnd").as_double(NAN));

                    for (auto [val_name, val_ptr] : {std::pair{"s0", &(road_object_repeat.s0)},
                                                     std::pair{"width_start", &(road_object_repeat.width_start)},
                                                     std::pair{"width_end", &(road_object_repeat.width_end)}})
                    {
                        odr::check_and_repair(
                            std::isnan((*val_ptr)) || (*val_ptr) >= 0,
                            [p = val_ptr]() { *p = 0; },
                            "Road #%s Object #%s: repeat::%s %f < 0, set %s=0",
                            road_id.c_str(),
                            road_object_id.c_str(),
                            val_name,
                            *val_ptr,
                            val_name);
                    }
                    for (auto [val_name, val_ptr] :
                         {std::pair{"length", &(road_object_repeat.length)}, std::pair{"distance", &(road_object_repeat.distance)}})
                    {
                        odr::check_and_repair((*val_ptr) >= 0,
                                              [p = val_ptr]() { *p = 0; },
                                              "Road #%s Object #%s: repeat::%s %f < 0, set %s=0",
                                              road_id.c_str(),
                                              road_object_id.c_str(),
                                              val_name,
                                              *val_ptr,
                                              val_name);
                    }

                    road_object.repeats.push_back(road_object_repeat);
                }

                // since v1.45 multiple <outline> are allowed and parent tag is <outlines>, not <object>; this supports v1.4 and v1.45+
                const pugi::xml_node outlines_parent_node = object_node.child("outlines") ? object_node.child("outlines") : object_node;
                for (const pugi::xml_node outline_node : outlines_parent_node.children("outline"))
                {
                    RoadObjectOutline road_object_outline(outline_node.attribute("id").as_int(-1),
                                                          outline_node.attribute("fillType").as_string(""),
                                                          outline_node.attribute("laneType").as_string(""),
                                                          outline_node.attribute("outer").as_bool(true),
                                                          outline_node.attribute("closed").as_bool(true));

                    for (const pugi::xml_node corner_local_node : outline_node.children("cornerLocal"))
                    {
                        const Vec3D pt_local{corner_local_node.attribute("u").as_double(0),
                                             corner_local_node.attribute("v").as_double(0),
                                             corner_local_node.attribute("z").as_double(0)};

                        RoadObjectCorner road_object_corner_local(corner_local_node.attribute("id").as_int(-1),
                                                                  pt_local,
                                                                  corner_local_node.attribute("height").as_double(0),
                                                                  default_local_outline_type);
                        road_object_outline.outline.push_back(road_object_corner_local);
                    }

                    for (const pugi::xml_node corner_road_node : outline_node.children("cornerRoad"))
                    {
                        const Vec3D pt_road{corner_road_node.attribute("s").as_double(0),
                                            corner_road_node.attribute("t").as_double(0),
                                            corner_road_node.attribute("dz").as_double(0)};

                        RoadObjectCorner road_object_corner_road(corner_road_node.attribute("id").as_int(-1),
                                                                 pt_road,
                                                                 corner_road_node.attribute("height").as_double(0),
                                                                 RoadObjectCorner::Type::Road);
                        road_object_outline.outline.push_back(road_object_corner_road);
                    }

                    road_object.outlines.push_back(road_object_outline);
                }

                road_object.lane_validities = extract_lane_validity_records(object_node);
            }
        }
        // parse signals
        if (with_road_signals)
        {
            for (const pugi::xml_node signal_node : road_node.child("signals").children("signal"))
            {
                std::string road_signal_id = signal_node.attribute("id").as_string("");
                odr::check_and_repair(
                    road.id_to_signal.find(road_signal_id) == road.id_to_signal.end(),
                    [&]() { road_signal_id = road_signal_id + std::string("_dup"); },
                    "Road #%s Signal #%s already exists, adding _dup suffix to id",
                    road_id.c_str(),
                    road_signal_id.c_str());

                RoadSignal& road_signal = road.id_to_signal
                                              .insert({road_signal_id,
                                                       RoadSignal(road_id,
                                                                  road_signal_id,
                                                                  signal_node.attribute("name").as_string(""),
                                                                  signal_node.attribute("s").as_double(0),
                                                                  signal_node.attribute("t").as_double(0),
                                                                  signal_node.attribute("dynamic").as_bool(),
                                                                  signal_node.attribute("zOffset").as_double(0),
                                                                  signal_node.attribute("value").as_double(0),
                                                                  signal_node.attribute("height").as_double(0),
                                                                  signal_node.attribute("width").as_double(0),
                                                                  signal_node.attribute("hOffset").as_double(0),
                                                                  signal_node.attribute("pitch").as_double(0),
                                                                  signal_node.attribute("roll").as_double(0),
                                                                  signal_node.attribute("orientation").as_string("none"),
                                                                  signal_node.attribute("country").as_string(""),
                                                                  signal_node.attribute("type").as_string("none"),
                                                                  signal_node.attribute("subtype").as_string("none"),
                                                                  signal_node.attribute("unit").as_string(""),
                                                                  signal_node.attribute("text").as_string("none"))})
                                              .first->second;

                for (auto [val_name, val_ptr] :
                     {std::pair{"s", &(road_signal.s0)}, std::pair{"height", &(road_signal.height)}, std::pair{"width", &(road_signal.width)}})
                {
                    odr::check_and_repair((*val_ptr) >= 0,
                                          [p = val_ptr]() { *p = 0; },
                                          "Road #%s Signal #%s: signal::%s %f < 0, set %s=0",
                                          road_id.c_str(),
                                          road_signal_id.c_str(),
                                          val_name,
                                          *val_ptr,
                                          val_name);
                }

                road_signal.lane_validities = extract_lane_validity_records(signal_node);
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
            log::error("Junction #%s already exists, skipping...", id.c_str());
            continue;
        }

        Junction& junction = this->id_to_junction.emplace(id, Junction(junction_node.attribute("name").as_string(""), id)).first->second;

        for (const pugi::xml_node connection_node : junction_node.children("connection"))
        {
            const std::string conn_id = connection_node.attribute("id").as_string("");
            if (junction.id_to_connection.find(conn_id) != junction.id_to_connection.end())
            {
                log::error("Junction #%s: discard already existing connection #%s", id.c_str(), conn_id.c_str());
                continue;
            }

            const std::string contact_point_str = connection_node.attribute("contactPoint").as_string("");
            if (!(contact_point_str == "start" || contact_point_str == "end"))
            {
                log::error(
                    "Junction #%s Connection #%s: discard due to unknown contactPoint '%s'", id.c_str(), conn_id.c_str(), contact_point_str.c_str());
                continue;
            }
            const JunctionConnection::ContactPoint contact_point =
                (contact_point_str == "start") ? JunctionConnection::ContactPoint::Start : JunctionConnection::ContactPoint::End;

            const std::string road_in = connection_node.attribute("incomingRoad").as_string("");
            const std::string road_conn = connection_node.attribute("connectingRoad").as_string("");
            if (this->id_to_road.find(road_in) == this->id_to_road.end())
            {
                log::warn("Junction #%s Connection #%s: discard due to incomingRoad #%s not found", id.c_str(), conn_id.c_str(), road_in.c_str());
                continue;
            }
            if (this->id_to_road.find(road_conn) == this->id_to_road.end())
            {
                log::warn(
                    "Junction #%s Connection #%s: discard due to connectingRoad road #%s not found", id.c_str(), conn_id.c_str(), road_conn.c_str());
                continue;
            }

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
            log::warn("Junction #%s: 0 connections", id.c_str());

        for (const pugi::xml_node priority_node : junction_node.children("priority"))
        {
            const std::string prio_high = priority_node.attribute("high").as_string("");
            const std::string prio_low = priority_node.attribute("low").as_string("");
            if (prio_low.empty() || prio_high.empty())
            {
                log::warn("Junction #%s: discard empty priority record high='%s', low='%s'", id.c_str(), prio_high.c_str(), prio_low.c_str());
                continue;
            }
            junction.priorities.emplace(prio_high, prio_low);
        }

        for (const pugi::xml_node controller_node : junction_node.children("controller"))
        {
            const std::string controller_id = controller_node.attribute("id").as_string("");
            if (junction.id_to_controller.find(controller_id) != junction.id_to_controller.end())
            {
                log::warn("Junction #%s: discard already existing controller #%s", id.c_str(), controller_id.c_str());
                continue;
            }
            const int64_t seq_id = controller_node.attribute("sequence").as_llong(-1); // type: uint32_t
            if (seq_id < 0)
            {
                log::warn("Junction #%s Controller #%s: discard due to sequence %ld < 0", id.c_str(), controller_id.c_str(), seq_id);
                continue;
            }
            const std::string controller_type = controller_node.attribute("type").as_string("");
            junction.id_to_controller.emplace(controller_id, JunctionController(controller_id, controller_type, seq_id));
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
