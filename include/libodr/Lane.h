#pragma once
#include "libodr/Geometries/CubicSpline.h"
#include "libodr/OdrNode.h"
#include "libodr/RoadMark.h"

#include <cstddef>
#include <functional>
#include <map>
#include <optional>
#include <ostream>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace odr
{

struct HeightOffset : public OdrNode
{
    HeightOffset(double s_offset, double inner, double outer);

    double s_offset;
    double inner;
    double outer;
};

struct LaneKey
{
    LaneKey(const std::string& road_id, double lane_section_s, int lane_id);
    std::string to_string() const;

    std::string road_id = "";
    double      lane_section_s = 0;
    int         lane_id = 0;

    bool operator==(const LaneKey& other) const
    {
        return this->road_id == other.road_id && this->lane_section_s == other.lane_section_s && this->lane_id == other.lane_id;
    }

    bool operator!=(const LaneKey& other) const
    {
        return !(*this == other);
    }
};

inline std::ostream& operator<<(std::ostream& os, const LaneKey& lk)
{
    return os << lk.to_string();
}

struct Lane : public OdrNode
{
    Lane(int id, std::optional<std::string> type = std::nullopt, std::optional<bool> level = std::nullopt);

    std::vector<SingleRoadMark> get_roadmarks(double s_start, double s_end) const;

    int id;

    std::optional<std::string> type; // required but can be treated as optional

    std::optional<bool> level;
    std::optional<int>  predecessor;
    std::optional<int>  successor;

    CubicProfile lane_width;
    CubicProfile outer_border;

    std::map<double, HeightOffset> s_to_height_offset;
    std::map<double, RoadMark>     s_to_road_mark;
};

} // namespace odr

namespace std
{
template<>
struct hash<odr::LaneKey>
{
    size_t operator()(const odr::LaneKey& key) const
    {
        return ((hash<string>()(key.road_id) ^ (hash<double>()(key.lane_section_s) << 1)) >> 1) ^ (hash<int>()(key.lane_id) << 1);
    }
};

template<>
struct equal_to<odr::LaneKey>
{
    bool operator()(const odr::LaneKey& lhs, const odr::LaneKey& rhs) const
    {
        return (lhs.road_id == rhs.road_id) && (lhs.lane_section_s == rhs.lane_section_s) && (lhs.lane_id == rhs.lane_id);
    }
};

template<>
struct less<odr::LaneKey>
{
    bool operator()(const odr::LaneKey& lhs, const odr::LaneKey& rhs) const
    {
        if (lhs.road_id != rhs.road_id)
            return lhs.road_id < rhs.road_id;
        if (lhs.lane_section_s != rhs.lane_section_s)
            return lhs.lane_section_s < rhs.lane_section_s;
        if (lhs.lane_id != rhs.lane_id)
            return lhs.lane_id < rhs.lane_id;
        return false;
    }
};
} // namespace std
