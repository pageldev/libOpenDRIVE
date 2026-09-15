#pragma once
#include "libodr/Math.hpp"
#include "libodr/RoadObject.h"

#include "fmt/core.h"
#include "magic_enum/magic_enum.hpp"
#include "pugixml.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace odr
{

template<class... Args>
inline void require_or_throw(bool ok, fmt::format_string<Args...> fmt, Args&&... args)
{
    if (!ok)
        throw std::runtime_error(fmt::format(fmt, std::forward<Args>(args)...));
}

template<typename T, typename Node>
std::conditional_t<std::is_const_v<Node>, const T, T>* get_parent_or_throw(Node& node)
{
    auto* parent = node.parent();
    require_or_throw(parent != nullptr, "node has no parent");
    using Parent = std::conditional_t<std::is_const_v<Node>, const T, T>;
    Parent* typed_parent = dynamic_cast<Parent*>(parent);
    require_or_throw(typed_parent != nullptr, "node has unexpected parent type");
    return typed_parent;
}

template<class K, class V>
std::set<K> get_map_keys(const std::map<K, V>& input_map)
{
    std::set<K> retval;
    std::transform(input_map.begin(), input_map.end(), std::inserter(retval, retval.end()), [](auto pair) { return pair.first; });
    return retval;
}

template<class V>
void insert_map_keys_if_in_range(std::set<double>& target_set, const std::map<double, V>& input_map, double a, double b)
{
    for (const auto& [s, _] : input_map)
    {
        if (s > a && s < b)
            target_set.insert(s);
    }
}

template<template<typename...> class Map, typename K, typename V>
V try_get_val(const Map<K, V>& m, const K& key, const V& default_val)
{
    auto iter = m.find(key);
    if (iter == m.end())
        return default_val;
    else
        return iter->second;
}

template<typename T, std::size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
std::vector<T> approximate_linear_quad_bezier(const std::array<Vec<T, Dim>, 3>& ctrl_pts, T eps)
{
    require_or_throw(std::isfinite(eps) && eps > 0, "eps must be finite and greater than 0 (got {})", eps);

    Vec<T, Dim> param_c;
    for (std::size_t dim = 0; dim < Dim; dim++)
        param_c[dim] = ctrl_pts[0][dim] - 2 * ctrl_pts[1][dim] + ctrl_pts[2][dim];

    const T step_size = std::min(std::sqrt((4 * eps) / norm(param_c)), 1.0);

    std::vector<T> p_vals;
    for (T p = 0; p < 1; p += step_size)
        p_vals.push_back(p);
    if (p_vals.back() != 1)
        p_vals.push_back(1);

    return p_vals;
}

template<typename T>
inline std::vector<T> get_triangle_strip_outline_indices(std::size_t num_vertices)
{
    if (num_vertices < 3)
        return {};

    std::vector<T> out_indices;
    out_indices.reserve(num_vertices + 4);

    for (std::size_t idx = 0; idx < num_vertices - 2; idx += 2)
    {
        out_indices.push_back(idx);
        out_indices.push_back(idx + 2);
    }
    for (std::size_t idx = 0 + 1; idx < num_vertices - 2; idx += 2)
    {
        out_indices.push_back(idx);
        out_indices.push_back(idx + 2);
    }

    out_indices.push_back(0);
    out_indices.push_back(1);
    out_indices.push_back(num_vertices - 2);
    out_indices.push_back(num_vertices - 1);

    return out_indices;
}

inline int next_towards_zero(int value)
{
    if (value > 0)
        return value - 1;
    else if (value < 0)
        return value + 1;
    return 0;
}

inline bool parse_bool(std::string_view s)
{
    return s == "1" || s == "true" || s == "yes";
}

template<typename T>
std::optional<T> try_get_attribute(const pugi::xml_node& node, const char* attr_name, bool treat_value_zero_as_missing = false)
{
    const auto attr = node.attribute(attr_name);
    if (!attr)
        return std::nullopt;

    const char* value = attr.value();
    if constexpr (std::is_same_v<T, std::string>)
    {
        return std::string(value);
    }
    else if constexpr (std::is_same_v<T, bool>)
    {
        return parse_bool(value);
    }
    else if constexpr (std::is_arithmetic_v<T>)
    {
        T           result{};
        const char* end = value + std::strlen(value);
        auto [ptr, ec] = std::from_chars(value, end, result);
        if (ec != std::errc{} || ptr != end)
            return std::nullopt;

        if (treat_value_zero_as_missing && result == T{})
            return std::nullopt;
        return result;
    }
    else
    {
        static_assert(std::is_same_v<T, void>, "unsupported T");
    }
}

template<typename T>
std::optional<T> try_get_enum(const pugi::xml_node node, const char* attr_name)
{
    std::optional<std::string> enum_str = try_get_attribute<std::string>(node, attr_name, false);
    if (!enum_str)
        return std::nullopt;
    return magic_enum::enum_cast<T>(*enum_str, magic_enum::case_insensitive);
}

inline std::optional<RoadObject::Orientation> try_get_orientation(const pugi::xml_node node, const char* attr_name)
{
    std::optional<std::string> orient_str = try_get_attribute<std::string>(node, attr_name, false);
    if (!orient_str)
        return std::nullopt;

    if (orient_str == "+")
        return RoadObject::Orientation::Positive;
    if (orient_str == "-")
        return RoadObject::Orientation::Negative;
    if (orient_str == "none")
        return RoadObject::Orientation::None;

    return std::nullopt;
};

inline bool is_zero(double x)
{
    return std::abs(x) < 1e-9;
}

template<typename T>
std::optional<int> find_first_gap_in_keys(const std::map<int, T>& map)
{
    const auto it = std::adjacent_find(map.begin(), map.end(), [](const auto& a, const auto& b) { return b.first != a.first + 1; });
    if (it == map.end())
        return std::nullopt;
    return it->first + 1;
}

} // namespace odr
