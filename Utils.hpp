#pragma once

#include "Math.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <type_traits>
#include <vector>

namespace odr
{
struct Box2D
{
    Box2D();
    Box2D(Vec2D min, Vec2D max);
    double get_distance(const Vec2D& pt);

    Vec2D  min, max;
    Vec2D  center;
    double width, height;
};

template<class C, class T, T C::*member>
struct SharedPtrCmp
{
    bool operator()(const std::shared_ptr<C>& lhs, const std::shared_ptr<C>& rhs) const { return (*lhs).*member < (*rhs).*member; }
};

template<typename T, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
Box2D get_bbox_for_s_values(const std::vector<T>& s_values, const std::function<Vec2D(T)>& get_xyz)
{
    std::vector<Vec2D> points;
    points.reserve(s_values.size());
    for (const T& s_val : s_values)
        points.push_back(get_xyz(s_val));

    auto iter_min_max_x = std::minmax_element(points.begin(), points.end(), [](const Vec2D& lhs, const Vec2D& rhs) { return lhs[0] < rhs[0]; });
    auto iter_min_max_y = std::minmax_element(points.begin(), points.end(), [](const Vec2D& lhs, const Vec2D& rhs) { return lhs[1] < rhs[1]; });

    Vec2D min = {iter_min_max_x.first->at(0), iter_min_max_y.first->at(1)};
    Vec2D max = {iter_min_max_x.second->at(0), iter_min_max_y.second->at(1)};

    return Box2D(min, max);
};

template<typename T, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
T golden_section_search(const std::function<T(T)>& f, T a, T b, const T tol)
{
    const T invphi = (std::sqrt(5) - 1) / 2;
    const T invphi2 = (3 - std::sqrt(5)) / 2;

    T h = b - a;
    if (h <= tol)
        return 0.5 * (a + b);

    // Required steps to achieve tolerance
    int n = static_cast<int>(std::ceil(std::log(tol / h) / std::log(invphi)));

    T c = a + invphi2 * h;
    T d = a + invphi * h;
    T yc = f(c);
    T yd = f(d);

    for (int k = 0; k < (n - 1); k++)
    {
        if (yc < yd)
        {
            b = d;
            d = c;
            yd = yc;
            h = invphi * h;
            c = a + invphi2 * h;
            yc = f(c);
        }
        else
        {
            a = c;
            c = d;
            yc = yd;
            h = invphi * h;
            d = a + invphi * h;
            yd = f(d);
        }
    }

    if (yc < yd)
        return 0.5 * (a + d);

    return 0.5 * (c + b);
}

inline bool get_lane_id_from_borders(double t, const std::map<int /*id*/, double /*outer brdr*/>& id_to_border, int& lane_id)
{
    /* returns false if point is out of road bounds - requires lane #0 to be present */
    for (auto iter = id_to_border.begin(); iter != id_to_border.end(); iter++)
    {
        const int    cur_lane_id = iter->first;
        const double outer_brdr = iter->second;

        if (cur_lane_id == 0 && id_to_border.at(0) == t)
        {
            lane_id = 0;
            return true;
        }
        else if (cur_lane_id < 0 && t >= outer_brdr && t < std::next(iter)->second)
        {
            lane_id = cur_lane_id;
            return true;
        }
        else if (cur_lane_id > 0 && t <= outer_brdr && t > std::prev(iter)->second)
        {
            lane_id = cur_lane_id;
            return true;
        }
    }

    return false;
}

} // namespace odr