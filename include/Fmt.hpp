#pragma once

#include <cstdio>
#include <stdexcept>
#include <string>

namespace odr
{

inline std::string strfmt(const char* fmt)
{
    return fmt ? std::string(fmt) : std::string();
}

template<class... Args, typename std::enable_if<(sizeof...(Args) > 0), int>::type = 0>
std::string strfmt(const char* fmt, Args&&... args)
{
    const int n = std::snprintf(nullptr, 0, fmt, std::forward<Args>(args)...);
    if (n < 0)
        throw std::runtime_error("formatting failed");
    std::string out(static_cast<size_t>(n), '\0'); // since c++11: str[str.size()] is '\0'
    std::snprintf(&out[0], out.size() + 1, fmt, std::forward<Args>(args)...);
    return out;
}

} // namespace odr