#pragma once
#include "Utils.hpp"

#include <atomic>
#include <cstdio>
#include <string>

namespace odr
{

enum LogLevel
{
    Info = 0,
    Warn = 1,
    Error = 2
};

inline const char* log_level_to_string(const LogLevel level)
{
    switch (level)
    {
    case LogLevel::Info:
        return "INFO";
    case LogLevel::Warn:
        return "WARN";
    case LogLevel::Error:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

using LogFunction = void (*)(LogLevel, const char*);

inline void default_log_function(LogLevel lvl, const char* msg)
{
    std::fprintf(stderr, "[%s] %s\n", log_level_to_string(lvl), msg);
}

inline std::atomic<LogFunction> g_log_function{&default_log_function};
inline std::atomic<LogLevel>    g_log_level{LogLevel::Warn};

inline void set_log_callback(LogFunction log_function)
{
    g_log_function.store(log_function, std::memory_order_relaxed);
}

inline void set_log_level(LogLevel lvl)
{
    g_log_level.store(lvl, std::memory_order_relaxed);
}

inline void log(LogLevel lvl, const char* msg)
{
    if (lvl < g_log_level.load(std::memory_order_relaxed))
        return;
    g_log_function.load(std::memory_order_relaxed)(lvl, msg);
}

template<class... Args>
void logf(LogLevel lvl, const char* fmt, Args&&... args)
{
    std::string s = strfmt(fmt, std::forward<Args>(args)...);
    log(lvl, s.c_str());
}

} // namespace odr
