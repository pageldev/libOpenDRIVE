#pragma once
#include "Fmt.hpp"

#include <atomic>
#include <cstdio>
#include <string>

namespace odr
{

namespace log
{

enum Level
{
    Info = 0,
    Warn = 1,
    Error = 2
};

inline const char* level_to_string(const Level level)
{
    switch (level)
    {
    case Level::Info:
        return "INFO";
    case Level::Warn:
        return "WARN";
    case Level::Error:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

using LogFunction = void (*)(Level, const char*);

inline void default_log_function(Level lvl, const char* msg)
{
    std::fprintf(stderr, "[%s] %s\n", level_to_string(lvl), msg);
}

inline std::atomic<LogFunction> g_log_function{&default_log_function};
inline std::atomic<Level>       g_log_level{Level::Warn};

inline void set_callback(LogFunction log_function)
{
    g_log_function.store(log_function, std::memory_order_relaxed);
}

inline void set_level(Level lvl)
{
    g_log_level.store(lvl, std::memory_order_relaxed);
}

template<class... Args>
void log(Level lvl, const char* fmt, Args&&... args)
{
    if (lvl < g_log_level.load(std::memory_order_relaxed))
        return;
    std::string s = strfmt(fmt, std::forward<Args>(args)...);
    g_log_function.load(std::memory_order_relaxed)(lvl, s.c_str());
}

template<class... Args>
void info(const char* fmt, Args&&... args)
{
    log(Level::Info, fmt, std::forward<Args>(args)...);
}

template<class... Args>
void warn(const char* fmt, Args&&... args)
{
    log(Level::Warn, fmt, std::forward<Args>(args)...);
}

template<class... Args>
void error(const char* fmt, Args&&... args)
{
    log(Level::Error, fmt, std::forward<Args>(args)...);
}

} // namespace log

} // namespace odr
