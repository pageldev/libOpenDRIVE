/**
 * Emscripten Embind bindings for libOpenDRIVE.
 *
 * ============================================================================
 * PURPOSE
 * ============================================================================
 * Provides JavaScript/TypeScript access to the libOpenDRIVE C++ API via
 * WebAssembly. This file is the ONLY bridge between the C++ geometry engine
 * and the TypeScript extension that renders OpenDRIVE maps in Lichtblick.
 *
 * Compiled with: emcmake cmake -DOPENDRIVE_BUILD_WASM=ON
 *   → produces libOpenDRIVE.js (ES module, SINGLE_FILE mode with embedded WASM)
 *
 * ============================================================================
 * WHY EMBIND
 * ============================================================================
 * Emscripten Embind (emscripten/bind.h) generates JavaScript wrapper code at
 * compile time that lets TypeScript call C++ classes and functions as if they
 * were native JS objects. This is preferred over raw cwrap/ccall because:
 *   1. Type-safe: C++ class hierarchies map to JS prototypes
 *   2. RAII-aware: .delete() releases C++ heap memory from JS
 *   3. STL support: register_vector/register_map expose std::vector/std::map
 *
 * ============================================================================
 * WHY HELPER FUNCTIONS (vs doing it in TypeScript)
 * ============================================================================
 * The helper functions (getLaneTypeMap, getRoadmarkColorMap, etc.) exist in C++
 * rather than TypeScript because they need to traverse the parsed OpenDRIVE
 * object model (Road → LaneSection → Lane → RoadMarkGroup etc.). Exposing the
 * full object graph via Embind would require binding dozens of classes and
 * containers. Instead, these helpers perform the traversal in C++ and return
 * flat map<vertex_start_idx, tab-separated-string> results that TypeScript can
 * consume with a simple .split("\t").
 *
 * This keeps the Embind surface minimal while giving TypeScript access to all
 * metadata needed for SceneEntity construction (entity IDs, colors, labels).
 *
 * ============================================================================
 * REFERENCES
 * ============================================================================
 * [libODR]     libOpenDRIVE — C++ library at submodule/libOpenDRIVE/
 *              https://github.com/lichtblick-suite/libOpenDRIVE
 * [EMB]        Emscripten Embind documentation:
 *              https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html
 * [ODR]        ASAM OpenDRIVE V1.8.1 specification (section references below)
 * [ODR §8.2]   Inertial coordinate system — all vertex output is (x, y, z)
 * [ODR §10]    Roads — reference line, predecessor/successor linkage
 * [ODR §11.1]  Lanes — lane ID numbering (negative = right, positive = left)
 * [ODR §11.3]  Lane sections — s-coordinate segmentation of the lane model
 * [ODR §11.7]  Lane types — e_laneType enumeration (driving, sidewalk, etc.)
 * [ODR §11.8]  Road markings — e_roadMarkColor and e_roadMarkType
 * [ODR §12]    Junctions — connecting roads, junction ID != "-1"
 * [ODR §13]    Road objects — barriers, poles, buildings, vegetation
 * [ODR §14]    Road signals — traffic signs, traffic lights
 * ============================================================================
 *
 * TypeScript counterpart: src/wasm/types.ts (interface definitions)
 * Consumer: src/converters/openDriveMap/sceneUpdateConverter.ts
 */

#ifdef __EMSCRIPTEN__

    #include "Junction.h"
    #include "Lane.h"
    #include "LaneSection.h"
    #include "Mesh.h"
    #include "OpenDriveMap.h"
    #include "RefLine.h"
    #include "Road.h"
    #include "RoadMark.h"
    #include "RoadNetworkMesh.h"
    #include "RoadObject.h"
    #include "RoadSignal.h"
    #include "Utils.hpp"

    #include <cstddef>
    #include <fstream>
    #include <map>
    #include <stdexcept>
    #include <string>
    #include <vector>

    #include <emscripten/bind.h>

namespace odr
{

/**
 * Factory: create OpenDriveMap from an XML string instead of a file path.
 *
 * WHY: In the browser there is no filesystem. The TypeScript extension receives
 * the OpenDRIVE XML as a UTF-8 string embedded in the osi3.MapAsamOpenDrive
 * protobuf message. We write it to Emscripten's in-memory virtual filesystem
 * (/tmp/) so the existing file-based OpenDriveMap constructor can parse it.
 *
 * @param xml_content   Complete OpenDRIVE XML document as a string
 * @param center_map    Subtract x/y centroid from all coordinates [libODR]
 * @param with_road_objects     Parse <object> elements [ODR §13]
 * @param with_lateral_profile  Apply superelevation & crossfall [ODR §9.5]
 * @param with_lane_height      Apply <height> offsets to lane surfaces [ODR §11.4.5]
 * @param abs_z_for_local_road_obj_outline  Use absolute z for object outlines
 * @param fix_spiral_edge_cases Handle spiral geometry numerical edge cases
 * @param with_road_signals     Parse <signal> elements [ODR §14]
 * @return Heap-allocated OpenDriveMap (caller owns; call .delete() from JS)
 */
OpenDriveMap* createFromXml(const std::string& xml_content,
                            bool               center_map,
                            bool               with_road_objects,
                            bool               with_lateral_profile,
                            bool               with_lane_height,
                            bool               abs_z_for_local_road_obj_outline,
                            bool               fix_spiral_edge_cases,
                            bool               with_road_signals)
{
    const std::string tmp_path = "/tmp/opendrive_input.xodr";
    {
        std::ofstream ofs(tmp_path);
        if (!ofs.is_open())
            throw std::runtime_error("Failed to open temporary file for OpenDRIVE XML");
        ofs << xml_content;
        if (!ofs.good())
            throw std::runtime_error("Failed to write OpenDRIVE XML to temporary file");
    }
    return new OpenDriveMap(tmp_path,
                            center_map,
                            with_road_objects,
                            with_lateral_profile,
                            with_lane_height,
                            abs_z_for_local_road_obj_outline,
                            fix_spiral_edge_cases,
                            with_road_signals);
}

/**
 * Get lane type [ODR §11.7] for each lane chunk in the LanesMesh.
 * Returns map<vertex_start_idx, lane_type_string>.
 *
 * WHY C++ side: Requires traversing Road → LaneSection → Lane to read
 * Lane::type. The mesh only stores vertex indices, not the parsed lane model.
 *
 * HOW: For each entry in lane_start_indices, resolve road_id + lanesec s0 +
 * lane_id from the mesh, look up the corresponding Lane in the parsed map,
 * and return its type string (e.g., "driving", "sidewalk").
 *
 * TypeScript consumer: sceneUpdateConverter.ts uses the type to pick lane
 * colors from LANE_COLORS and to label SceneEntity metadata.
 */
std::map<std::size_t, std::string> getLaneTypeMap(const OpenDriveMap& odr_map, const LanesMesh& mesh)
{
    std::map<std::size_t, std::string> result;

    for (const auto& [vert_idx, lane_id] : mesh.lane_start_indices)
    {
        const std::string road_id = mesh.get_road_id(vert_idx);
        const double      s0 = mesh.get_lanesec_s0(vert_idx);

        auto road_it = odr_map.id_to_road.find(road_id);
        if (road_it == odr_map.id_to_road.end())
            continue;

        const auto& lanesecs = road_it->second.s_to_lanesection;
        auto        ls_it = lanesecs.lower_bound(s0 - 1e-6);
        if (ls_it == lanesecs.end())
            continue;

        auto lane_it = ls_it->second.id_to_lane.find(lane_id);
        if (lane_it != ls_it->second.id_to_lane.end())
        {
            result[vert_idx] = lane_it->second.type;
        }
    }
    return result;
}

/**
 * Get IDs of roads that belong to a junction [ODR §12].
 * A road is a junction connecting road if Road::junction != "-1".
 *
 * WHY: TypeScript uses this to apply a distinct "junction" color tint
 * to lane surfaces inside junctions, making intersection topology visible.
 */
std::vector<std::string> getJunctionRoadIds(const OpenDriveMap& odr_map)
{
    std::vector<std::string> result;
    for (const auto& [id, road] : odr_map.id_to_road)
    {
        if (!road.junction.empty() && road.junction != "-1")
        {
            result.push_back(id);
        }
    }
    return result;
}

/**
 * Get roadmark color [ODR §11.8] for each roadmark chunk in the RoadmarksMesh.
 * Returns map<vertex_start_idx, color_string> (e.g., "white", "yellow").
 *
 * WHY C++ side: Requires traversing Road → LaneSection → Lane → RoadMarkGroup
 * to read RoadMarkGroup::color. The mesh only has mark type, not color.
 *
 * HOW: For each roadmark_type_start_indices entry, walk the parsed lane's
 * roadmark_groups to find a matching type. If the type match has a color,
 * use it; otherwise fall back to the first group with any non-empty color.
 *
 * TypeScript consumer: sceneUpdateConverter.ts maps the color string to
 * ROAD_MARK_COLORS for TriangleListPrimitive rendering.
 */
std::map<std::size_t, std::string> getRoadmarkColorMap(const OpenDriveMap& odr_map, const RoadmarksMesh& mesh)
{
    std::map<std::size_t, std::string> result;

    for (const auto& [vert_idx, mark_type] : mesh.roadmark_type_start_indices)
    {
        const std::string road_id = mesh.get_road_id(vert_idx);
        const double      s0 = mesh.get_lanesec_s0(vert_idx);
        const int         lane_id = mesh.get_lane_id(vert_idx);

        auto road_it = odr_map.id_to_road.find(road_id);
        if (road_it == odr_map.id_to_road.end())
            continue;

        const auto& lanesecs = road_it->second.s_to_lanesection;
        auto        ls_it = lanesecs.lower_bound(s0 - 1e-6);
        if (ls_it == lanesecs.end())
            continue;

        auto lane_it = ls_it->second.id_to_lane.find(lane_id);
        if (lane_it == ls_it->second.id_to_lane.end())
            continue;

        for (const auto& rmg : lane_it->second.roadmark_groups)
        {
            if (rmg.type == mark_type && !rmg.color.empty())
            {
                result[vert_idx] = rmg.color;
                break;
            }
        }
        // Fallback: use first group with a non-empty color
        if (result.find(vert_idx) == result.end())
        {
            for (const auto& rmg : lane_it->second.roadmark_groups)
            {
                if (!rmg.color.empty())
                {
                    result[vert_idx] = rmg.color;
                    break;
                }
            }
        }
    }
    return result;
}

/**
 * Get road metadata [ODR §10] for each road chunk in a LanesMesh.
 * Returns map<vertex_start_idx, tab-separated metadata string>:
 *   "name\tlength\tjunction\tspeed_max\tspeed_unit\ttype"
 *
 * WHY C++ side: Road-level attributes (name, length, junction, speed, type)
 * live in the parsed Road object, not in the mesh. Exposing Road via Embind
 * would require binding its full interface; a flat string is simpler.
 *
 * HOW: Iterates road_start_indices, looks up each Road by ID, reads its
 * attributes, and caches per road_id to avoid redundant lookups (multiple
 * lane chunks share the same road).
 *
 * TypeScript consumer: sceneUpdateConverter.ts splits the string and stores
 * the fields as SceneEntity metadata key-value pairs.
 */
std::map<std::size_t, std::string> getRoadMetadataMap(const OpenDriveMap& odr_map, const LanesMesh& mesh)
{
    std::map<std::size_t, std::string> result;
    std::map<std::string, std::string> road_meta_cache;

    for (const auto& [vert_idx, road_id_val] : mesh.road_start_indices)
    {
        const std::string& road_id = road_id_val;
        auto               cache_it = road_meta_cache.find(road_id);
        if (cache_it != road_meta_cache.end())
        {
            result[vert_idx] = cache_it->second;
            continue;
        }

        auto road_it = odr_map.id_to_road.find(road_id);
        if (road_it == odr_map.id_to_road.end())
            continue;

        const Road& road = road_it->second;

        // Find the speed record at s=0 (first applicable speed)
        std::string speed_max = "";
        std::string speed_unit = "";
        if (!road.s_to_speed.empty())
        {
            const auto& sp = road.s_to_speed.begin()->second;
            speed_max = sp.max;
            speed_unit = sp.unit;
        }

        // Find the road type at s=0
        std::string road_type = "";
        if (!road.s_to_type.empty())
        {
            road_type = road.s_to_type.begin()->second;
        }

        std::string meta =
            road.name + "\t" + std::to_string(road.length) + "\t" + road.junction + "\t" + speed_max + "\t" + speed_unit + "\t" + road_type;
        road_meta_cache[road_id] = meta;
        result[vert_idx] = meta;
    }
    return result;
}

/**
 * Get road object metadata [ODR §13] for each object chunk in RoadObjectsMesh.
 * Returns map<vertex_start_idx, tab-separated metadata string>:
 *   "type\tname\tsubtype\torientation\tis_dynamic\twidth\theight\tlength\ts0\tt0"
 *
 * WHY C++ side: Object attributes (type, dimensions, position) are in the
 * parsed RoadObject, not the mesh. The mesh only contains the generated
 * triangles and their grouping by object ID.
 *
 * TypeScript consumer: sceneUpdateConverter.ts uses these fields for
 * SceneEntity metadata (tooltip labels in the 3D panel).
 */
std::map<std::size_t, std::string> getRoadObjectMetadataMap(const OpenDriveMap& odr_map, const RoadObjectsMesh& mesh)
{
    std::map<std::size_t, std::string> result;

    for (const auto& [vert_idx, obj_id] : mesh.road_object_start_indices)
    {
        const std::string road_id = mesh.get_road_id(vert_idx);

        auto road_it = odr_map.id_to_road.find(road_id);
        if (road_it == odr_map.id_to_road.end())
            continue;

        auto obj_it = road_it->second.id_to_object.find(obj_id);
        if (obj_it == road_it->second.id_to_object.end())
            continue;

        const RoadObject& obj = obj_it->second;
        result[vert_idx] = obj.type + "\t" + obj.name + "\t" + obj.subtype + "\t" + obj.orientation + "\t" + (obj.is_dynamic ? "true" : "false") +
                           "\t" + std::to_string(obj.width) + "\t" + std::to_string(obj.height) + "\t" + std::to_string(obj.length) + "\t" +
                           std::to_string(obj.s0) + "\t" + std::to_string(obj.t0);
    }
    return result;
}

/**
 * Get road signal metadata [ODR §14] for each signal chunk in RoadSignalsMesh.
 * Returns map<vertex_start_idx, tab-separated metadata string>:
 *   "name\tcountry\ttype\tsubtype\tvalue\ttext\tis_dynamic\theight\twidth\torientation"
 *
 * WHY C++ side: Signal attributes (country code, type/subtype classification,
 * dynamic flag) live in the parsed RoadSignal object.
 *
 * TypeScript consumer: sceneUpdateConverter.ts uses these for SceneEntity
 * metadata (e.g., "type=1000001" → traffic light in German catalog).
 */
std::map<std::size_t, std::string> getRoadSignalMetadataMap(const OpenDriveMap& odr_map, const RoadSignalsMesh& mesh)
{
    std::map<std::size_t, std::string> result;

    for (const auto& [vert_idx, sig_id] : mesh.road_signal_start_indices)
    {
        const std::string road_id = mesh.get_road_id(vert_idx);

        auto road_it = odr_map.id_to_road.find(road_id);
        if (road_it == odr_map.id_to_road.end())
            continue;

        auto sig_it = road_it->second.id_to_signal.find(sig_id);
        if (sig_it == road_it->second.id_to_signal.end())
            continue;

        const RoadSignal& sig = sig_it->second;
        result[vert_idx] = sig.name + "\t" + sig.country + "\t" + sig.type + "\t" + sig.subtype + "\t" + std::to_string(sig.value) + "\t" + sig.text +
                           "\t" + (sig.is_dynamic ? "true" : "false") + "\t" + std::to_string(sig.height) + "\t" + std::to_string(sig.width) + "\t" +
                           sig.orientation;
    }
    return result;
}

/**
 * Get roadmark group metadata [ODR §11.8] for each roadmark chunk.
 * Returns map<vertex_start_idx, tab-separated metadata string>:
 *   "type\tweight\tlane_change\twidth"
 *
 * WHY: RoadMarkGroup attributes (weight, lane_change permission, width) are
 * only available in the parsed Lane model, not in the mesh.
 *
 * TypeScript consumer: sceneUpdateConverter.ts stores these as SceneEntity
 * metadata for road marking tooltip inspection.
 */
std::map<std::size_t, std::string> getRoadmarkMetadataMap(const OpenDriveMap& odr_map, const RoadmarksMesh& mesh)
{
    std::map<std::size_t, std::string> result;

    for (const auto& [vert_idx, mark_type] : mesh.roadmark_type_start_indices)
    {
        const std::string road_id = mesh.get_road_id(vert_idx);
        const double      s0 = mesh.get_lanesec_s0(vert_idx);
        const int         lane_id = mesh.get_lane_id(vert_idx);

        auto road_it = odr_map.id_to_road.find(road_id);
        if (road_it == odr_map.id_to_road.end())
            continue;

        const auto& lanesecs = road_it->second.s_to_lanesection;
        auto        ls_it = lanesecs.lower_bound(s0 - 1e-6);
        if (ls_it == lanesecs.end())
            continue;

        auto lane_it = ls_it->second.id_to_lane.find(lane_id);
        if (lane_it == ls_it->second.id_to_lane.end())
            continue;

        for (const auto& rmg : lane_it->second.roadmark_groups)
        {
            if (rmg.type == mark_type)
            {
                result[vert_idx] = rmg.type + "\t" + rmg.weight + "\t" + rmg.lane_change + "\t" + std::to_string(rmg.width);
                break;
            }
        }
        // Fallback: use first group if type match failed
        if (result.find(vert_idx) == result.end() && !lane_it->second.roadmark_groups.empty())
        {
            const auto& rmg = *lane_it->second.roadmark_groups.begin();
            result[vert_idx] = rmg.type + "\t" + rmg.weight + "\t" + rmg.lane_change + "\t" + std::to_string(rmg.width);
        }
    }
    return result;
}

/**
 * Get road predecessor/successor linkage [ODR §10.3] for each road.
 * Returns map<vertex_start_idx, tab-separated linkage string>:
 *   "pred_id\tpred_type\tpred_contact\tsucc_id\tsucc_type\tsucc_contact"
 *
 * WHY: Road topology (which road connects to which at start/end) is needed
 * for SceneEntity metadata so users can inspect road connectivity in the
 * 3D panel. The mesh has no topology awareness — only vertex data.
 *
 * HOW: Iterates road_start_indices, reads Road::predecessor and
 * Road::successor, serializes type (road/junction) and contact point
 * (start/end). Caches per road_id since multiple lane chunks share a road.
 */
std::map<std::size_t, std::string> getRoadLinkageMap(const OpenDriveMap& odr_map, const LanesMesh& mesh)
{
    std::map<std::size_t, std::string> result;
    std::map<std::string, std::string> link_cache;

    auto contact_str = [](RoadLink::ContactPoint cp) -> std::string
    {
        switch (cp)
        {
        case RoadLink::ContactPoint::Start:
            return "start";
        case RoadLink::ContactPoint::End:
            return "end";
        default:
            return "";
        }
    };

    auto type_str = [](RoadLink::Type t) -> std::string
    {
        switch (t)
        {
        case RoadLink::Type::Road:
            return "road";
        case RoadLink::Type::Junction:
            return "junction";
        default:
            return "";
        }
    };

    for (const auto& [vert_idx, road_id_val] : mesh.road_start_indices)
    {
        auto cache_it = link_cache.find(road_id_val);
        if (cache_it != link_cache.end())
        {
            result[vert_idx] = cache_it->second;
            continue;
        }

        auto road_it = odr_map.id_to_road.find(road_id_val);
        if (road_it == odr_map.id_to_road.end())
            continue;

        const Road& road = road_it->second;

        std::string meta = road.predecessor.id + "\t" + type_str(road.predecessor.type) + "\t" + contact_str(road.predecessor.contact_point) + "\t" +
                           road.successor.id + "\t" + type_str(road.successor.type) + "\t" + contact_str(road.successor.contact_point);
        link_cache[road_id_val] = meta;
        result[vert_idx] = meta;
    }
    return result;
}

/**
 * Get lane predecessor/successor IDs [ODR §11.1] for each lane chunk.
 * Returns map<vertex_start_idx, "predecessor_lane_id\tsuccessor_lane_id">.
 *
 * WHY: Lane-level connectivity (which lane connects to which across lane
 * section boundaries) is useful for understanding lane continuity in the
 * 3D panel metadata. Lane::predecessor/successor are optional<int>.
 */
std::map<std::size_t, std::string> getLaneLinkageMap(const OpenDriveMap& odr_map, const LanesMesh& mesh)
{
    std::map<std::size_t, std::string> result;

    for (const auto& [vert_idx, lane_id] : mesh.lane_start_indices)
    {
        const std::string road_id = mesh.get_road_id(vert_idx);
        const double      s0 = mesh.get_lanesec_s0(vert_idx);

        auto road_it = odr_map.id_to_road.find(road_id);
        if (road_it == odr_map.id_to_road.end())
            continue;

        const auto& lanesecs = road_it->second.s_to_lanesection;
        auto        ls_it = lanesecs.lower_bound(s0 - 1e-6);
        if (ls_it == lanesecs.end())
            continue;

        auto lane_it = ls_it->second.id_to_lane.find(lane_id);
        if (lane_it == ls_it->second.id_to_lane.end())
            continue;

        const Lane& lane = lane_it->second;
        std::string pred = lane.predecessor.has_value() ? std::to_string(lane.predecessor.value()) : "";
        std::string succ = lane.successor.has_value() ? std::to_string(lane.successor.value()) : "";
        result[vert_idx] = pred + "\t" + succ;
    }
    return result;
}

/**
 * ============================================================================
 * EMSCRIPTEN_BINDINGS — Class/function registration
 * ============================================================================
 * This block tells Embind which C++ types and functions to expose to JavaScript.
 * Each registration generates JS glue code at compile time.
 *
 * MEMORY MODEL:
 *   - Objects created in JS (via `new Module.OpenDriveMap(...)`) live on the
 *     C++ heap. They MUST be freed by calling `.delete()` from JS.
 *   - Calling `.delete()` twice on the same object crashes the WASM runtime
 *     (Emscripten does not guard against double-free).
 *   - register_vector/register_map create JS wrappers around C++ STL containers.
 *     Their `.delete()` only releases the JS wrapper; the underlying data is
 *     owned by the parent object's lifetime.
 *
 * HIERARCHY:
 *   Mesh3D → RoadsMesh → LanesMesh → RoadmarksMesh
 *                       → RoadObjectsMesh
 *                       → RoadSignalsMesh
 *   Each level adds grouping indices (road → lane section → lane → roadmark).
 * ============================================================================
 */
EMSCRIPTEN_BINDINGS(libOpenDRIVE)
{
    /* ─── Value arrays ─────────────────────────────────────────────── */
    /* Map C++ std::array / Vec2D / Vec3D to JS arrays [EMB value_array] */
    emscripten::value_array<std::array<std::size_t, 2>>("array_size_t_2").element(emscripten::index<0>()).element(emscripten::index<1>());
    emscripten::value_array<Vec2D>("Vec2D").element(emscripten::index<0>()).element(emscripten::index<1>());
    emscripten::value_array<Vec3D>("Vec3D").element(emscripten::index<0>()).element(emscripten::index<1>()).element(emscripten::index<2>());

    /* ─── Vectors ──────────────────────────────────────────────────── */
    /* Expose std::vector<T> as JS objects with .size(), .get(), .push_back(), .delete() */
    emscripten::register_vector<std::size_t>("vector_size_t");
    emscripten::register_vector<std::uint32_t>("vector_uint32_t");
    emscripten::register_vector<int>("vector_int");
    emscripten::register_vector<double>("vector_double");
    emscripten::register_vector<Vec2D>("vector_Vec2D");
    emscripten::register_vector<Vec3D>("vector_Vec3D");
    emscripten::register_vector<std::string>("vector_string");
    emscripten::register_vector<Mesh3D>("vector_Mesh3D");

    /* ─── Maps ─────────────────────────────────────────────────────── */
    /* Expose std::map<K,V> as JS objects with .size(), .get(), .keys(), .delete() */
    emscripten::register_map<std::size_t, std::string>("map_size_t_string");
    emscripten::register_map<std::size_t, double>("map_size_t_double");
    emscripten::register_map<std::size_t, int>("map_size_t_int");

    /* ─── Mesh3D ───────────────────────────────────────────────────── */
    /* Base triangle mesh [libODR Mesh.h]: vertices + triangle indices + normals
     * All coordinates are in OpenDRIVE inertial frame [ODR §8.2] */
    emscripten::class_<Mesh3D>("Mesh3D")
        .constructor<>()
        .function("get_obj", &Mesh3D::get_obj)
        .property("vertices", &Mesh3D::vertices)
        .property("indices", &Mesh3D::indices)
        .property("normals", &Mesh3D::normals)
        .property("st_coordinates", &Mesh3D::st_coordinates);

    /* ─── RoadsMesh ────────────────────────────────────────────────── */
    /* Adds per-road vertex grouping via road_start_indices [ODR §10] */
    emscripten::class_<RoadsMesh, emscripten::base<Mesh3D>>("RoadsMesh")
        .function("get_road_id", &RoadsMesh::get_road_id)
        .function("get_idx_interval_road", &RoadsMesh::get_idx_interval_road)
        .property("road_start_indices", &RoadsMesh::road_start_indices);

    /* ─── LanesMesh ────────────────────────────────────────────────── */
    /* Adds lane section [ODR §11.3] and lane [ODR §11.1] grouping.
     * get_lane_outline_indices() returns LINE_LIST pairs for lane boundary rendering */
    emscripten::class_<LanesMesh, emscripten::base<RoadsMesh>>("LanesMesh")
        .function("get_lanesec_s0", &LanesMesh::get_lanesec_s0)
        .function("get_lane_id", &LanesMesh::get_lane_id)
        .function("get_idx_interval_lanesec", &LanesMesh::get_idx_interval_lanesec)
        .function("get_idx_interval_lane", &LanesMesh::get_idx_interval_lane)
        .function("get_lane_outline_indices", &LanesMesh::get_lane_outline_indices)
        .property("lanesec_start_indices", &LanesMesh::lanesec_start_indices)
        .property("lane_start_indices", &LanesMesh::lane_start_indices);

    /* ─── RoadmarksMesh ────────────────────────────────────────────── */
    /* Adds per-roadmark-type grouping [ODR §11.8] for colored marking rendering */
    emscripten::class_<RoadmarksMesh, emscripten::base<LanesMesh>>("RoadmarksMesh")
        .function("get_roadmark_type", &RoadmarksMesh::get_roadmark_type)
        .function("get_idx_interval_roadmark", &RoadmarksMesh::get_idx_interval_roadmark)
        .function("get_roadmark_outline_indices", &RoadmarksMesh::get_roadmark_outline_indices)
        .property("roadmark_type_start_indices", &RoadmarksMesh::roadmark_type_start_indices);

    /* ─── RoadObjectsMesh ──────────────────────────────────────────── */
    /* Per-road-object grouping [ODR §13]: barriers, poles, buildings */
    emscripten::class_<RoadObjectsMesh, emscripten::base<RoadsMesh>>("RoadObjectsMesh")
        .function("get_road_object_id", &RoadObjectsMesh::get_road_object_id)
        .function("get_idx_interval_road_object", &RoadObjectsMesh::get_idx_interval_road_object)
        .property("road_object_start_indices", &RoadObjectsMesh::road_object_start_indices);

    /* ─── RoadSignalsMesh ──────────────────────────────────────────── */
    /* Per-road-signal grouping [ODR §14]: traffic signs, traffic lights */
    emscripten::class_<RoadSignalsMesh, emscripten::base<RoadsMesh>>("RoadSignalsMesh")
        .function("get_road_signal_id", &RoadSignalsMesh::get_road_signal_id)
        .function("get_idx_interval_signal", &RoadSignalsMesh::get_idx_interval_signal)
        .property("road_signal_start_indices", &RoadSignalsMesh::road_signal_start_indices);

    /* ─── RoadNetworkMesh ──────────────────────────────────────────── */
    /* Top-level mesh container returned by get_road_network_mesh(eps).
     * Contains all sub-meshes; this is the main TypeScript entry point. */
    emscripten::class_<RoadNetworkMesh>("RoadNetworkMesh")
        .function("get_mesh", &RoadNetworkMesh::get_mesh)
        .property("lanes_mesh", &RoadNetworkMesh::lanes_mesh)
        .property("roadmarks_mesh", &RoadNetworkMesh::roadmarks_mesh)
        .property("road_objects_mesh", &RoadNetworkMesh::road_objects_mesh)
        .property("road_signals_mesh", &RoadNetworkMesh::road_signals_mesh);

    /* ─── OpenDriveMap ─────────────────────────────────────────────── */
    /* Main class: parses OpenDRIVE XML → internal road model → mesh generation.
     * Constructor takes xodr file path + feature flags.
     * get_road_network_mesh(eps) performs adaptive tessellation. */
    emscripten::class_<OpenDriveMap>("OpenDriveMap")
        .constructor<std::string, bool, bool, bool, bool, bool, bool, bool>()
        .function("get_road_network_mesh", &OpenDriveMap::get_road_network_mesh)
        .function("get_roads", &OpenDriveMap::get_roads)
        .function("get_junctions", &OpenDriveMap::get_junctions)
        .property("proj4", &OpenDriveMap::proj4)
        .property("x_offs", &OpenDriveMap::x_offs)
        .property("y_offs", &OpenDriveMap::y_offs)
        .property("xodr_file", &OpenDriveMap::xodr_file);

    /* ─── Free functions ───────────────────────────────────────────── */
    /* These are the helper functions defined above. They take an OpenDriveMap
     * reference and a mesh reference, traverse the parsed road model, and
     * return flat map<vertex_start_idx, tab-separated-string> results.
     * createFromXml uses allow_raw_pointers because it returns a heap-allocated
     * OpenDriveMap* that Embind wraps as a JS-owned pointer. */
    emscripten::function("createFromXml", &createFromXml, emscripten::allow_raw_pointers());
    emscripten::function("getLaneTypeMap", &getLaneTypeMap);
    emscripten::function("getJunctionRoadIds", &getJunctionRoadIds);
    emscripten::function("getRoadmarkColorMap", &getRoadmarkColorMap);
    emscripten::function("getRoadMetadataMap", &getRoadMetadataMap);
    emscripten::function("getRoadObjectMetadataMap", &getRoadObjectMetadataMap);
    emscripten::function("getRoadSignalMetadataMap", &getRoadSignalMetadataMap);
    emscripten::function("getRoadmarkMetadataMap", &getRoadmarkMetadataMap);
    emscripten::function("getRoadLinkageMap", &getRoadLinkageMap);
    emscripten::function("getLaneLinkageMap", &getLaneLinkageMap);
}

} // namespace odr

#endif
