#include "libodr/OpenDriveMap.h"
#include "fmt/format.h"
#include "libodr/Geometries/Arc.h"
#include "libodr/Geometries/CubicSpline.h"
#include "libodr/Geometries/Line.h"
#include "libodr/Geometries/ParamPoly3.h"
#include "libodr/Geometries/RoadGeometry.h"
#include "libodr/Geometries/Spiral.h"
#include "libodr/Junction.h"
#include "libodr/Lane.h"
#include "libodr/LaneSection.h"
#include "libodr/LaneValidityRecord.h"
#include "libodr/Math.hpp"
#include "libodr/RefLine.h"
#include "libodr/Road.h"
#include "libodr/RoadMark.h"
#include "libodr/RoadObject.h"
#include "libodr/RoadSignal.h"
#include "libodr/Utils.hpp"

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

OpenDriveMapHeader::OpenDriveMapHeader(std::optional<int>         rev_major,
                                       std::optional<int>         rev_minor,
                                       std::optional<double>      north,
                                       std::optional<double>      east,
                                       std::optional<double>      south,
                                       std::optional<double>      west,
                                       std::optional<std::string> date,
                                       std::optional<std::string> name,
                                       std::optional<std::string> vendor,
                                       std::optional<std::string> version,
                                       std::optional<std::string> proj) :
    rev_major(rev_major),
    rev_minor(rev_minor),
    north(north),
    east(east),
    south(south),
    west(west),
    date(date),
    name(name),
    vendor(vendor),
    version(version),
    proj(proj)
{
}

XodrParseResult OpenDriveMap::load(const pugi::xml_document& xml_doc,
                                   const bool                with_road_objects,
                                   const bool                with_lateral_profile,
                                   const bool                with_lane_height,
                                   const bool                abs_z_for_for_local_road_obj_outline,
                                   const bool                fix_spiral_edge_cases,
                                   const bool                with_road_signals,
                                   const bool                treat_value_zero_as_missing)
{
    XodrParseResult      result;
    const pugi::xml_node odr_node = xml_doc.child("OpenDRIVE");

    const pugi::xml_node header_node = odr_node.child("header");
    const pugi::xml_node georef_node = header_node.child("geoReference");

    this->header = OpenDriveMapHeader(try_get_attribute<int>(header_node, "revMajor"),
                                      try_get_attribute<int>(header_node, "revMinor"),
                                      try_get_attribute<double>(header_node, "north"),
                                      try_get_attribute<double>(header_node, "east"),
                                      try_get_attribute<double>(header_node, "south"),
                                      try_get_attribute<double>(header_node, "west"),
                                      try_get_attribute<std::string>(header_node, "date"),
                                      try_get_attribute<std::string>(header_node, "name"),
                                      try_get_attribute<std::string>(header_node, "vendor"),
                                      try_get_attribute<std::string>(header_node, "version"),
                                      georef_node ? std::optional<std::string>(georef_node.text().as_string("")) : std::nullopt);

    // Roads
    if (!odr_node.child("road"))
        result.errors.push_back({odr_node, "no roads found"});
    for (const pugi::xml_node road_node : odr_node.children("road"))
    {
        const std::optional<std::string> road_id = try_get_attribute<std::string>(road_node, "id");
        if (!road_id)
        {
            result.errors.push_back({road_node, "missing id"});
            continue;
        }
        if (this->id_to_road.find(*road_id) != this->id_to_road.end())
        {
            result.errors.push_back({road_node, "duplicate id"});
            continue;
        }

        std::optional<Road> road;
        try
        {
            road.emplace(*road_id,
                         road_node.attribute("length").as_double(NAN),
                         road_node.attribute("junction").as_string("-1"), // -1 for none
                         try_get_enum<Road::TrafficRule>(road_node, "rule"),
                         try_get_attribute<std::string>(road_node, "name"));
        }
        catch (const std::exception& ex)
        {
            result.errors.push_back({road_node, ex.what()});
            continue;
        }

        // parse road links
        const pugi::xml_node link_node = road_node.child("link");
        for (const bool is_predecessor : {true, false})
        {
            const pugi::xml_node next_link_node = is_predecessor ? link_node.child("predecessor") : link_node.child("successor");
            if (next_link_node)
            {
                std::optional<RoadLink> link;
                try
                {
                    link.emplace(next_link_node.attribute("elementId").as_string(""),
                                 next_link_node.attribute("elementType").as_string(""),
                                 try_get_enum<RoadLink::ContactPoint>(next_link_node, "contactPoint"));
                }
                catch (const std::exception& ex)
                {
                    result.errors.push_back({next_link_node, ex.what()});
                    continue;
                }

                std::optional<RoadLink>& road_link = is_predecessor ? road->predecessor : road->successor;
                road_link = link;
            }
        }
        if (const pugi::xml_node neighbor_node = link_node.child("neighbor"))
            result.errors.push_back({neighbor_node, "<neighbor> not supported"});

        // parse road type and speed
        for (const pugi::xml_node road_type_node : road_node.children("type"))
        {
            const double      s = road_type_node.attribute("s").as_double(NAN);
            const std::string type = road_type_node.attribute("type").as_string("");
            if (s < 0)
            {
                result.errors.push_back({road_type_node, "s < 0"});
                continue;
            }
            road->s_to_type[s] = type;

            if (const pugi::xml_node node = road_type_node.child("speed"))
            {
                const std::string speed_record_max = node.attribute("max").as_string("");
                const std::string speed_record_unit = node.attribute("unit").as_string("");
                road->s_to_speed.emplace(s, SpeedRecord(speed_record_max, speed_record_unit));
            }
        }

        // make ref_line - parse road geometries
        bool                 invalid_geometry = false;
        const pugi::xml_node plan_view_node = road_node.child("planView");
        for (const pugi::xml_node geometry_hdr_node : plan_view_node.children("geometry"))
        {
            const double s0 = geometry_hdr_node.attribute("s").as_double(NAN);
            const double hdg0 = geometry_hdr_node.attribute("hdg").as_double(NAN);
            const double length = geometry_hdr_node.attribute("length").as_double(NAN);
            double       x0 = geometry_hdr_node.attribute("x").as_double(NAN);
            double       y0 = geometry_hdr_node.attribute("y").as_double(NAN);

            const pugi::xml_node geometry_node = geometry_hdr_node.first_child();
            const std::string    geometry_type = geometry_node.name();
            try
            {
                if (geometry_type == "line")
                {
                    road->ref_line.s0_to_geometry[s0] = std::make_unique<Line>(s0, x0, y0, hdg0, length);
                }
                else if (geometry_type == "spiral")
                {
                    const double curv_start = geometry_node.attribute("curvStart").as_double(NAN);
                    const double curv_end = geometry_node.attribute("curvEnd").as_double(NAN);
                    if (!fix_spiral_edge_cases)
                    {
                        road->ref_line.s0_to_geometry[s0] = std::make_unique<Spiral>(s0, x0, y0, hdg0, length, curv_start, curv_end);
                    }
                    else
                    {
                        if (std::abs(curv_start) < 1e-6 && std::abs(curv_end) < 1e-6)
                        {
                            // In effect a line
                            road->ref_line.s0_to_geometry[s0] = std::make_unique<Line>(s0, x0, y0, hdg0, length);
                        }
                        else if (std::abs(curv_end - curv_start) < 1e-6)
                        {
                            // In effect an arc
                            road->ref_line.s0_to_geometry[s0] = std::make_unique<Arc>(s0, x0, y0, hdg0, length, curv_start);
                        }
                        else
                        {
                            // True spiral
                            road->ref_line.s0_to_geometry[s0] = std::make_unique<Spiral>(s0, x0, y0, hdg0, length, curv_start, curv_end);
                        }
                    }
                }
                else if (geometry_type == "arc")
                {
                    const double curvature = geometry_node.attribute("curvature").as_double(NAN);
                    road->ref_line.s0_to_geometry[s0] = std::make_unique<Arc>(s0, x0, y0, hdg0, length, curvature);
                }
                else if (geometry_type == "paramPoly3")
                {
                    const std::optional<ParamPoly3::PRange> p_range_geom = try_get_enum<ParamPoly3::PRange>(geometry_node, "pRange");
                    const std::optional<ParamPoly3::PRange> p_range_hdr = try_get_enum<ParamPoly3::PRange>(geometry_hdr_node, "pRange");

                    // pRange from <paramPoly3> takes precedence over <geometry, default to 'normalized'
                    const ParamPoly3::PRange p_range = p_range_geom ? *p_range_geom : p_range_hdr.value_or(ParamPoly3::PRange::Normalized);
                    road->ref_line.s0_to_geometry[s0] = std::make_unique<ParamPoly3>(s0,
                                                                                     x0,
                                                                                     y0,
                                                                                     hdg0,
                                                                                     length,
                                                                                     geometry_node.attribute("aU").as_double(NAN),
                                                                                     geometry_node.attribute("bU").as_double(NAN),
                                                                                     geometry_node.attribute("cU").as_double(NAN),
                                                                                     geometry_node.attribute("dU").as_double(NAN),
                                                                                     geometry_node.attribute("aV").as_double(NAN),
                                                                                     geometry_node.attribute("bV").as_double(NAN),
                                                                                     geometry_node.attribute("cV").as_double(NAN),
                                                                                     geometry_node.attribute("dV").as_double(NAN),
                                                                                     p_range);
                }
                else
                {
                    result.errors.push_back({geometry_node, "unknown geometry"});
                    invalid_geometry = true;
                    continue;
                }
            }
            catch (const std::exception& ex)
            {
                result.errors.push_back({geometry_hdr_node, ex.what()});
                invalid_geometry = true;
                continue;
            }
        }
        if (road->ref_line.s0_to_geometry.empty())
        {
            result.errors.push_back({plan_view_node ? plan_view_node : road_node, "no geometries"});
            invalid_geometry = true;
        }
        if (invalid_geometry)
            continue; // discard road

        std::map<std::string /*x path query*/, CubicProfile&> cubic_profile_fields{
            {".//elevationProfile//elevation", road->ref_line.elevation_profile}, {".//lanes//laneOffset", road->lane_offset}};
        if (with_lateral_profile)
            cubic_profile_fields.insert({".//lateralProfile//superelevation", road->superelevation});

        // parse elevation profiles, lane offsets, superelevation
        bool invalid_cubic = false;
        for (auto& [xpath_query_str, cubic_profile] : cubic_profile_fields)
        {
            pugi::xpath_node_set xnodes = road_node.select_nodes(xpath_query_str.c_str());
            for (pugi::xpath_node xnode : xnodes)
            {
                const pugi::xml_node node = xnode.node();
                const double         s0 = node.attribute("s").as_double(NAN);

                std::optional<CubicPoly> cubic_poly;
                try
                {
                    require_or_throw(s0 >= 0, "s {} < 0", s0);
                    cubic_poly.emplace(node.attribute("a").as_double(NAN),
                                       node.attribute("b").as_double(NAN),
                                       node.attribute("c").as_double(NAN),
                                       node.attribute("d").as_double(NAN),
                                       s0);
                }
                catch (const std::exception& ex)
                {
                    result.errors.push_back({node, ex.what()});
                    invalid_cubic = true;
                    continue;
                }

                cubic_profile.segments.emplace(s0, *cubic_poly);
            }
        }
        if (invalid_cubic)
            continue; // discard road

        // parse crossfall - has extra attribute side
        const pugi::xml_node lateral_profile_node = road_node.child("lateralProfile");
        if (with_lateral_profile)
        {
            for (const pugi::xml_node crossfall_node : lateral_profile_node.children("crossfall"))
            {
                const double s0 = crossfall_node.attribute("s").as_double(NAN);

                std::optional<CubicPoly> crossfall_poly;
                try
                {
                    require_or_throw(s0 >= 0, "s {} < 0", s0);
                    crossfall_poly.emplace(crossfall_node.attribute("a").as_double(NAN),
                                           crossfall_node.attribute("b").as_double(NAN),
                                           crossfall_node.attribute("c").as_double(NAN),
                                           crossfall_node.attribute("d").as_double(NAN),
                                           s0);
                }
                catch (const std::exception& ex)
                {
                    result.errors.push_back({crossfall_node, ex.what()});
                    continue;
                }

                road->crossfall.segments.emplace(s0, *crossfall_poly);
                const std::optional<Crossfall::Side> side = try_get_enum<Crossfall::Side>(crossfall_node, "side");
                road->crossfall.s_to_side[s0] = side.value_or(Crossfall::Side::Both); // default to 'both'
            }

            if (const pugi::xml_node shape_node = lateral_profile_node.child("shape"))
                result.errors.push_back({shape_node, "<shape> not supported"});
        }

        // parse road lane sections and lanes
        bool invalid_lanesection = false;
        for (const pugi::xml_node lanesection_node : road_node.child("lanes").children("laneSection"))
        {
            std::optional<LaneSection> lanesection;
            try
            {
                lanesection.emplace(lanesection_node.attribute("s").as_double(NAN));
            }
            catch (const std::exception& ex)
            {
                result.errors.push_back({lanesection_node, ex.what()});
                invalid_lanesection = true;
                continue;
            }

            for (const pugi::xpath_node lane_xpath_node : lanesection_node.select_nodes(".//lane"))
            {
                const pugi::xml_node     lane_node = lane_xpath_node.node();
                const std::optional<int> lane_id = try_get_attribute<int>(lane_node, "id");
                if (!lane_id)
                {
                    result.errors.push_back({lane_node, "missing id"});
                    continue;
                }
                if (lanesection->id_to_lane.find(*lane_id) != lanesection->id_to_lane.end())
                {
                    result.errors.push_back({lane_node, "duplicate id"});
                    continue;
                }

                if (const pugi::xml_node border_node = lane_node.child("border"))
                    result.errors.push_back({border_node, "<border> not supported"});

                Lane& lane =
                    lanesection->id_to_lane
                        .emplace(*lane_id,
                                 Lane(*lane_id, try_get_attribute<std::string>(lane_node, "type"), try_get_attribute<bool>(lane_node, "level")))
                        .first->second;

                if (const pugi::xml_attribute id_attr = lane_node.child("link").child("predecessor").attribute("id"))
                    lane.predecessor = id_attr.as_int();
                if (const pugi::xml_attribute id_attr = lane_node.child("link").child("successor").attribute("id"))
                    lane.successor = id_attr.as_int();

                for (const pugi::xml_node lane_width_node : lane_node.children("width"))
                {
                    const double s_offset = lane_width_node.attribute("sOffset").as_double(NAN);

                    std::optional<CubicPoly> width_poly;
                    try
                    {
                        require_or_throw(s_offset >= 0, "sOffset {} < 0", s_offset);
                        width_poly.emplace(lane_width_node.attribute("a").as_double(NAN),
                                           lane_width_node.attribute("b").as_double(NAN),
                                           lane_width_node.attribute("c").as_double(NAN),
                                           lane_width_node.attribute("d").as_double(NAN),
                                           lanesection->s0 + s_offset);
                    }
                    catch (const std::exception& ex)
                    {
                        result.errors.push_back({lane_width_node, ex.what()});
                        invalid_lanesection = true;
                        continue;
                    }

                    // OpenDRIVE Format Specification, Rev. 1.4, 3.3.1 General:
                    // "The reference line itself is defined as lane zero and must not have a width entry (i.e. its width must always be 0.0)."
                    if (lane_id == 0 && !width_poly->is_zero())
                    {
                        result.errors.push_back({lane_width_node, "width must be 0 for lane #0"});
                        width_poly->set_zero();
                    }

                    lane.lane_width.segments.emplace(lanesection->s0 + s_offset, *width_poly);
                }

                if (with_lane_height)
                {
                    for (const pugi::xml_node lane_height_node : lane_node.children("height"))
                    {
                        std::optional<HeightOffset> height_offset;
                        try
                        {
                            height_offset.emplace(lane_height_node.attribute("sOffset").as_double(NAN),
                                                  lane_height_node.attribute("inner").as_double(NAN),
                                                  lane_height_node.attribute("outer").as_double(NAN)

                            );
                        }
                        catch (const std::exception& ex)
                        {
                            result.errors.push_back({lane_height_node, ex.what()});
                            continue;
                        }
                        lane.s_to_height_offset.emplace(lanesection->s0 + height_offset->s_offset, *height_offset);
                    }
                }

                for (const pugi::xml_node roadmark_node : lane_node.children("roadMark"))
                {
                    std::optional<RoadMark> roadmark;
                    try
                    {
                        roadmark.emplace(roadmark_node.attribute("sOffset").as_double(NAN),
                                         roadmark_node.attribute("type").as_string(""),
                                         roadmark_node.attribute("color").as_string(""),
                                         try_get_attribute<double>(roadmark_node, "width"),
                                         try_get_attribute<double>(roadmark_node, "height", treat_value_zero_as_missing),
                                         try_get_attribute<std::string>(roadmark_node, "weight"),
                                         try_get_attribute<std::string>(roadmark_node, "material"),
                                         try_get_attribute<std::string>(roadmark_node, "laneChange"));
                    }
                    catch (const std::exception& ex)
                    {
                        result.errors.push_back({roadmark_node, ex.what()});
                        continue;
                    }

                    if (const pugi::xml_node roadmark_type_node = roadmark_node.child("type"))
                    {
                        std::optional<RoadMarkType> roadmark_type;
                        try
                        {
                            roadmark_type.emplace(roadmark_type_node.attribute("name").as_string(""),
                                                  try_get_attribute<double>(roadmark_type_node, "width", treat_value_zero_as_missing));
                        }
                        catch (const std::exception& ex)
                        {
                            result.errors.push_back({roadmark_type_node, ex.what()});
                        }

                        if (roadmark_type)
                        {
                            for (const pugi::xml_node roadmarks_line_node : roadmark_type_node.children("line"))
                            {
                                std::optional<RoadMarkLine> roadmark_line;
                                try
                                {
                                    roadmark_line.emplace(roadmarks_line_node.attribute("sOffset").as_double(NAN),
                                                          roadmarks_line_node.attribute("tOffset").as_double(NAN),
                                                          roadmarks_line_node.attribute("length").as_double(NAN),
                                                          try_get_attribute<double>(roadmarks_line_node, "width", treat_value_zero_as_missing),
                                                          try_get_attribute<double>(roadmarks_line_node, "space"),
                                                          try_get_attribute<std::string>(roadmarks_line_node, "color"),
                                                          try_get_attribute<std::string>(roadmarks_line_node, "rule"));
                                }
                                catch (const std::exception& ex)
                                {
                                    result.errors.push_back({roadmarks_line_node, ex.what()});
                                    continue;
                                }

                                roadmark_type->lines.emplace_back(std::move(*roadmark_line));
                            }
                            roadmark->type_elem = roadmark_type;
                        }
                    }

                    lane.s_to_roadmark.emplace(lanesection->s0 + roadmark->s_offset, std::move(*roadmark));
                }
            }

            // derive lane borders from lane widths
            const auto id_lane_iter0 = lanesection->id_to_lane.find(0);
            if (id_lane_iter0 == lanesection->id_to_lane.end())
            {
                result.errors.push_back({lanesection_node, "missing lane #0"});
                invalid_lanesection = true;
                continue;
            }

            // iterate from lane #1 towards +inf
            const auto id_lane_iter1 = std::next(id_lane_iter0);
            for (auto iter = id_lane_iter1; iter != lanesection->id_to_lane.end(); iter++)
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
            for (auto r_iter = r_id_lane_iter1; r_iter != lanesection->id_to_lane.rend(); r_iter++)
            {
                if (r_iter == r_id_lane_iter1)
                    r_iter->second.outer_border = r_iter->second.lane_width.negate();
                else
                    r_iter->second.outer_border = std::prev(r_iter)->second.outer_border.add(r_iter->second.lane_width.negate());
            }

            // OpenDRIVE Format Specification, Rev. 1.4, 3.3.2 Lane Offset:
            // "... lane 0 may be offset using a cubic polynom"
            for (auto& id_lane : lanesection->id_to_lane)
                id_lane.second.outer_border = id_lane.second.outer_border.add(road->lane_offset);

            road->s_to_lanesection.emplace(lanesection->s0, std::move(*lanesection));
        }
        if (invalid_lanesection)
            continue; // discard road

        // parse road objects
        if (with_road_objects)
        {
            const RoadObjectCorner::Type default_local_outline_type =
                abs_z_for_for_local_road_obj_outline ? RoadObjectCorner::Type::Local_AbsZ : RoadObjectCorner::Type::Local_RelZ;

            for (const pugi::xml_node object_node : road_node.child("objects").children("object"))
            {
                const std::optional<std::string> object_id = try_get_attribute<std::string>(object_node, "id");
                if (!object_id)
                {
                    result.errors.push_back({object_node, "missing id"});
                    continue;
                }
                if (road->id_to_object.find(*object_id) != road->id_to_object.end())
                {
                    result.errors.push_back({object_node, "duplicate id"});
                    continue;
                }

                std::optional<RoadObject> road_object;
                try
                {
                    road_object.emplace(*object_id,
                                        try_get_attribute<double>(object_node, "s"),
                                        try_get_attribute<double>(object_node, "t"),
                                        try_get_attribute<double>(object_node, "zOffset"),
                                        try_get_attribute<double>(object_node, "length", treat_value_zero_as_missing),
                                        try_get_attribute<double>(object_node, "validLength"),
                                        try_get_attribute<double>(object_node, "width"),
                                        try_get_attribute<double>(object_node, "radius", treat_value_zero_as_missing),
                                        try_get_attribute<double>(object_node, "height"),
                                        try_get_attribute<double>(object_node, "hdg"),
                                        try_get_attribute<double>(object_node, "pitch"),
                                        try_get_attribute<double>(object_node, "roll"),
                                        try_get_attribute<std::string>(object_node, "type"),
                                        try_get_attribute<std::string>(object_node, "name"),
                                        try_get_attribute<std::string>(object_node, "subtype"),
                                        try_get_orientation(object_node, "orientation"),
                                        try_get_attribute<bool>(object_node, "dynamic"));
                }
                catch (const std::exception& ex)
                {
                    result.errors.push_back({object_node, ex.what()});
                    continue;
                }

                for (const pugi::xml_node repeat_node : object_node.children("repeat"))
                {
                    try
                    {
                        road_object->repeats.emplace_back(repeat_node.attribute("s").as_double(NAN),
                                                          repeat_node.attribute("length").as_double(NAN),
                                                          repeat_node.attribute("distance").as_double(NAN),
                                                          try_get_attribute<double>(repeat_node, "tStart"),
                                                          try_get_attribute<double>(repeat_node, "tEnd"),
                                                          try_get_attribute<double>(repeat_node, "heightStart"),
                                                          try_get_attribute<double>(repeat_node, "heightEnd"),
                                                          try_get_attribute<double>(repeat_node, "zOffsetStart"),
                                                          try_get_attribute<double>(repeat_node, "zOffsetEnd"),
                                                          try_get_attribute<double>(repeat_node, "widthStart"),
                                                          try_get_attribute<double>(repeat_node, "widthEnd"));
                    }
                    catch (const std::exception& ex)
                    {
                        result.errors.push_back({repeat_node, ex.what()});
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
                            result.errors.push_back({corner_local_node, ex.what()});
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
                            result.errors.push_back({corner_road_node, ex.what()});
                        }
                    }

                    road_object->outlines.push_back(std::move(road_object_outline));
                }

                for (const pugi::xml_node validity_node : object_node.children("validity"))
                {
                    const std::optional<int> from_lane = try_get_attribute<int>(validity_node, "fromLane");
                    const std::optional<int> to_lane = try_get_attribute<int>(validity_node, "toLane");

                    if (!from_lane || !to_lane)
                    {
                        result.errors.push_back({validity_node, "required attributes 'fromLane' or 'toLane' missing"});
                        continue;
                    }
                    road_object->lane_validities.emplace_back(*from_lane, *to_lane);
                }

                road->id_to_object.emplace(*object_id, std::move(*road_object));
            }
        }
        // parse signals
        if (with_road_signals)
        {
            for (const pugi::xml_node signal_node : road_node.child("signals").children("signal"))
            {
                const std::optional<std::string> signal_id = try_get_attribute<std::string>(signal_node, "id");
                if (!signal_id)
                {
                    result.errors.push_back({signal_node, "missing id"});
                    continue;
                }
                if (road->id_to_signal.find(*signal_id) != road->id_to_signal.end())
                {
                    result.errors.push_back({signal_node, "duplicate id"});
                    continue;
                }

                std::optional<RoadSignal> road_signal;
                try
                {
                    road_signal.emplace(*signal_id,
                                        *road_id,
                                        signal_node.attribute("s").as_double(NAN),
                                        signal_node.attribute("t").as_double(NAN),
                                        signal_node.attribute("zOffset").as_double(0.0), // default to 0, often treated as optional
                                        signal_node.attribute("dynamic").as_bool(),
                                        signal_node.attribute("type").as_string("none"),
                                        signal_node.attribute("subtype").as_string("none"),
                                        try_get_orientation(signal_node, "orientation").value_or(RoadObject::Orientation::None),
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
                    result.errors.push_back({signal_node, ex.what()});
                    continue;
                }

                for (const pugi::xml_node validity_node : signal_node.children("validity"))
                {
                    if (!(validity_node.attribute("fromLane") && validity_node.attribute("toLane")))
                    {
                        result.errors.push_back({validity_node, "required attributes 'fromLane' or 'toLane' missing"});
                        continue;
                    }
                    const int from_lane = validity_node.attribute("fromLane").as_int(INT_MIN);
                    const int to_lane = validity_node.attribute("toLane").as_int(INT_MAX);
                    road_signal->lane_validities.emplace_back(from_lane, to_lane);
                }

                road->id_to_signal.emplace(*signal_id, std::move(*road_signal));
            }
        }

        this->id_to_road.emplace(road->id, std::move(*road));
    }

    // Junctions
    for (const pugi::xml_node junction_node : odr_node.children("junction"))
    {
        const std::optional<std::string> junction_id = try_get_attribute<std::string>(junction_node, "id");
        if (!junction_id)
        {
            result.errors.push_back({junction_node, "missing id"});
            continue;
        }
        if (this->id_to_junction.find(*junction_id) != this->id_to_junction.end())
        {
            result.errors.push_back({junction_node, "duplicate id"});
            continue;
        }

        Junction& junction =
            this->id_to_junction.emplace(*junction_id, Junction(*junction_id, try_get_attribute<std::string>(junction_node, "name"))).first->second;

        for (const pugi::xml_node connection_node : junction_node.children("connection"))
        {
            const std::optional<std::string> conn_id = try_get_attribute<std::string>(connection_node, "id");
            if (!conn_id)
            {
                result.errors.push_back({junction_node, "missing id"});
                continue;
            }
            if (junction.id_to_connection.find(*conn_id) != junction.id_to_connection.end())
            {
                result.errors.push_back({connection_node, "duplicate id"});
                continue;
            }

            JunctionConnection& connection = junction.id_to_connection
                                                 .emplace(*conn_id,
                                                          JunctionConnection(*conn_id,
                                                                             connection_node.attribute("incomingRoad").as_string(""),
                                                                             connection_node.attribute("connectingRoad").as_string(""),
                                                                             connection_node.attribute("contactPoint").as_string("")))
                                                 .first->second;

            for (const pugi::xml_node lane_link_node : connection_node.children("laneLink"))
            {
                const std::optional<int> from_lane = try_get_attribute<int>(lane_link_node, "from");
                const std::optional<int> to_lane = try_get_attribute<int>(lane_link_node, "to");
                if (!from_lane || !to_lane)
                {
                    result.errors.push_back({lane_link_node, "required attributes 'from' or 'to' missing"});
                    continue;
                }
                connection.lane_links.emplace(*from_lane, *to_lane);
            }
        }

        const std::size_t num_conns = junction.id_to_connection.size();
        if (num_conns == 0)
            result.errors.push_back({junction_node, "no connections"});

        for (const pugi::xml_node priority_node : junction_node.children("priority"))
        {
            const std::optional<std::string> prio_high = try_get_attribute<std::string>(priority_node, "high");
            const std::optional<std::string> prio_low = try_get_attribute<std::string>(priority_node, "low");

            if (!prio_low || !prio_high)
            {
                result.errors.push_back({priority_node, "required attributes 'low' or 'high' missing"});
                continue;
            }
            junction.priorities.emplace(*prio_high, *prio_low);
        }

        for (const pugi::xml_node controller_node : junction_node.children("controller"))
        {
            const std::optional<std::string> controller_id = try_get_attribute<std::string>(controller_node, "id");
            if (!controller_id)
            {
                result.errors.push_back({controller_node, "missing id"});
                continue;
            }
            if (junction.id_to_controller.find(*controller_id) != junction.id_to_controller.end())
            {
                result.errors.push_back({controller_node, "duplicate id"});
                continue;
            }

            std::optional<JunctionController> junction_controller;
            try
            {
                junction_controller.emplace(
                    *controller_id, try_get_attribute<std::string>(controller_node, "type"), try_get_attribute<int64_t>(controller_node, "sequence"));
            }
            catch (const std::exception& ex)
            {
                result.errors.push_back({controller_node, ex.what()});
                continue;
            }

            junction.id_to_controller.emplace(*controller_id, std::move(*junction_controller));
        }
    }

    return result;
}

void OpenDriveMap::reset()
{
    *this = OpenDriveMap{};
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

RoadNetworkMesh OpenDriveMap::get_road_network_mesh(const double eps) const
{
    RoadNetworkMesh  out_mesh;
    LanesMesh&       lanes_mesh = out_mesh.lanes_mesh;
    RoadmarksMesh&   roadmarks_mesh = out_mesh.roadmarks_mesh;
    RoadObjectsMesh& road_objects_mesh = out_mesh.road_objects_mesh;
    RoadSignalsMesh& road_signals_mesh = out_mesh.road_signals_mesh;

    for (const auto& [road_id, road] : this->id_to_road)
    {
        lanes_mesh.road_start_indices[lanes_mesh.vertices.size()] = road_id;
        roadmarks_mesh.road_start_indices[roadmarks_mesh.vertices.size()] = road_id;
        road_objects_mesh.road_start_indices[road_objects_mesh.vertices.size()] = road_id;

        for (const auto& [lanesec_s0, lanesec] : road.s_to_lanesection)
        {
            lanes_mesh.lanesec_start_indices[lanes_mesh.vertices.size()] = lanesec_s0;
            roadmarks_mesh.lanesec_start_indices[roadmarks_mesh.vertices.size()] = lanesec_s0;
            for (const auto& [lane_id, lane] : lanesec.id_to_lane)
            {
                const LaneKey     lane_key(road_id, lanesec_s0, lane_id);
                const std::size_t lanes_idx_offset = lanes_mesh.vertices.size();
                lanes_mesh.lane_start_indices[lanes_idx_offset] = lane_id;
                lanes_mesh.add_mesh(road.get_lane_mesh(lane_key, eps));

                std::size_t roadmarks_idx_offset = roadmarks_mesh.vertices.size();
                roadmarks_mesh.lane_start_indices[roadmarks_idx_offset] = lane_id;
                const std::vector<SingleRoadMark> roadmarks = lane.get_roadmarks(lanesec.s0, road.get_lanesection_end(lanesec));
                for (const SingleRoadMark& roadmark : roadmarks)
                {
                    roadmarks_idx_offset = roadmarks_mesh.vertices.size();
                    roadmarks_mesh.roadmark_type_start_indices[roadmarks_idx_offset] = roadmark.type;
                    roadmarks_mesh.add_mesh(road.get_roadmark_mesh(lane_key, roadmark, eps));
                }
            }
        }

        for (const auto& [road_object_id, road_object] : road.id_to_object)
        {
            const std::size_t road_objs_idx_offset = road_objects_mesh.vertices.size();
            road_objects_mesh.road_object_start_indices[road_objs_idx_offset] = road_object_id;
            road_objects_mesh.add_mesh(road.get_road_object_mesh(road_object, eps));
        }

        for (const auto& [road_signal_id, road_signal] : road.id_to_signal)
        {
            const std::size_t signals_idx_offset = road_signals_mesh.vertices.size();
            road_signals_mesh.road_signal_start_indices[signals_idx_offset] = road_signal_id;
            road_signals_mesh.add_mesh(road.get_road_signal_mesh(road_signal));
        }
    }

    return out_mesh;
}

RoutingGraph OpenDriveMap::get_routing_graph(std::vector<std::string>* errors) const
{
    RoutingGraph routing_graph;

    // helper function, only uses road successor/predecessor links, no junction links
    auto get_next_lane_basic = [this](const LaneKey& lane, int next_lane_id, bool predecessor) -> std::optional<LaneKey>
    {
        const auto id_road_iter = this->id_to_road.find(lane.road_id);
        if (id_road_iter == this->id_to_road.end())
            return std::nullopt;

        const Road& road = id_road_iter->second;
        const auto  s_lanesec_iter = road.s_to_lanesection.find(lane.lanesection_s0);
        if (s_lanesec_iter == road.s_to_lanesection.end()) // also catches empty road
            return std::nullopt;

        // next lanesection in the same road
        if (predecessor)
        {
            if (s_lanesec_iter != road.s_to_lanesection.begin())
            {
                const LaneSection& prev_lanesec = std::prev(s_lanesec_iter)->second;
                if (prev_lanesec.id_to_lane.find(next_lane_id) != prev_lanesec.id_to_lane.end())
                    return LaneKey(lane.road_id, prev_lanesec.s0, next_lane_id);
            }
        }
        else
        {
            const auto next_lanesec_iter = std::next(s_lanesec_iter);
            if (next_lanesec_iter != road.s_to_lanesection.end())
            {
                const LaneSection& next_lanesec = next_lanesec_iter->second;
                if (next_lanesec.id_to_lane.find(next_lane_id) != next_lanesec.id_to_lane.end())
                    return LaneKey(lane.road_id, next_lanesec.s0, next_lane_id);
            }
        }

        // next lanesection NOT in the same road
        const std::optional<RoadLink>& road_link = predecessor ? road.predecessor : road.successor;
        if (road_link && road_link->type == RoadLink::Type::Road)
        {
            const auto next_road_iter = this->id_to_road.find(road_link->id);
            if (next_road_iter == this->id_to_road.end())
                return std::nullopt;

            const Road& next_road = next_road_iter->second;
            if (next_road.s_to_lanesection.empty())
                return std::nullopt;

            const LaneSection& next_lanesection = (*(road_link->contact_point) == RoadLink::ContactPoint::Start) // Road always has ContactPoint
                                                      ? next_road.s_to_lanesection.begin()->second
                                                      : next_road.s_to_lanesection.rbegin()->second;
            if (next_lanesection.id_to_lane.find(next_lane_id) != next_lanesection.id_to_lane.end())
                return LaneKey(next_road.id, next_lanesection.s0, next_lane_id);
        }

        return std::nullopt;
    };

    // Parse Roads
    for (const auto& [road_id, road] : id_to_road)
    {
        for (const auto& [lanesection_s0, lanesection] : road.s_to_lanesection)
        {
            for (const auto& [lane_id, lane] : lanesection.id_to_lane)
            {
                const LaneKey lane_key(road_id, lanesection_s0, lane_id);
                const bool    lane_follows_road_dir = lane_id < 0;

                if (lane.predecessor)
                {
                    const std::optional<LaneKey> predecessor_lane = get_next_lane_basic(lane_key, *(lane.predecessor), lane_follows_road_dir);
                    if (predecessor_lane)
                    {
                        const Road&  predecessor_road = this->id_to_road.at(predecessor_lane->road_id);
                        const double lane_length = predecessor_road.get_lanesection_length(predecessor_lane->lanesection_s0);
                        routing_graph.add_edge(RoutingGraphEdge(*predecessor_lane, lane_key, lane_length));
                    }
                }

                if (lane.successor)
                {
                    const std::optional<LaneKey> successor_lane = get_next_lane_basic(lane_key, *(lane.successor), !lane_follows_road_dir);
                    if (successor_lane)
                    {
                        const double lane_length = road.get_lanesection_length(lane_key.lanesection_s0);
                        routing_graph.add_edge(RoutingGraphEdge(lane_key, *successor_lane, lane_length));
                    }
                }
            }
        }
    }

    // Parse Junctions
    for (const auto& [junc_id, junction] : id_to_junction)
    {
        for (const auto& [conn_id, conn] : junction.id_to_connection)
        {
            const std::string _loc_str = fmt::format("/junction[@id={}]/connection[@id={}]", junc_id, conn_id);

            auto road_in_iter = id_to_road.find(conn.incoming_road);
            if (road_in_iter == id_to_road.end())
            {
                if (errors)
                    errors->push_back(fmt::format("{}: incoming road[@id={}] not found", _loc_str, conn.incoming_road));
                continue;
            }
            auto road_conn_iter = id_to_road.find(conn.connecting_road);
            if (road_conn_iter == id_to_road.end())
            {
                if (errors)
                    errors->push_back(fmt::format("{}: connecting road[@id={}] not found", _loc_str, conn.connecting_road));
                continue;
            }

            const Road&        road_in = road_in_iter->second;
            const LaneSection& incoming_lanesec = road_in.s_to_lanesection.rbegin()->second;
            const Road&        road_conn = road_conn_iter->second;
            const LaneSection& connecting_lanesec = (conn.contact_point == JunctionConnection::ContactPoint::Start)
                                                        ? road_conn.s_to_lanesection.begin()->second
                                                        : road_conn.s_to_lanesection.rbegin()->second;

            for (const JunctionLaneLink& lane_link : conn.lane_links)
            {
                auto from_lane_iter = incoming_lanesec.id_to_lane.find(lane_link.from);
                auto to_lane_iter = connecting_lanesec.id_to_lane.find(lane_link.to);

                if (from_lane_iter == incoming_lanesec.id_to_lane.end() || to_lane_iter == connecting_lanesec.id_to_lane.end())
                    continue;

                const LaneKey from(road_in.id, incoming_lanesec.s0, from_lane_iter->second.id);
                const LaneKey to(road_conn.id, connecting_lanesec.s0, to_lane_iter->second.id);
                const double  lane_length = road_in.get_lanesection_length(incoming_lanesec);

                routing_graph.add_edge(RoutingGraphEdge(from, to, lane_length));
            }
        }
    }

    return routing_graph;
}

} // namespace odr
