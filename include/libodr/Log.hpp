#pragma once

#include <atomic>
#include <cstdio>
#include <fmt/format.h>
#include <string>

namespace odr
{

namespace log
{

enum class Level
{
    Debug = 0,
    Info = 1,
    Warn = 2,
    Error = 3
};

inline const char* level_to_string(const Level level)
{
    switch (level)
    {
    case Level::Debug:
        return "DEBUG";
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
void log(Level lvl, fmt::format_string<Args...> fmt, Args&&... args)
{
    if (lvl < g_log_level.load(std::memory_order_relaxed))
        return;
    std::string s = fmt::format(fmt, std::forward<Args>(args)...);
    g_log_function.load(std::memory_order_relaxed)(lvl, s.c_str());
}

template<class... Args>
void debug(fmt::format_string<Args...> fmt, Args&&... args)
{
    log(Level::Debug, fmt, std::forward<Args>(args)...);
}

template<class... Args>
void info(fmt::format_string<Args...> fmt, Args&&... args)
{
    log(Level::Info, fmt, std::forward<Args>(args)...);
}

template<class... Args>
void warn(fmt::format_string<Args...> fmt, Args&&... args)
{
    log(Level::Warn, fmt, std::forward<Args>(args)...);
}

template<class... Args>
void error(fmt::format_string<Args...> fmt, Args&&... args)
{
    log(Level::Error, fmt, std::forward<Args>(args)...);
}

} // namespace log

} // namespace odr
