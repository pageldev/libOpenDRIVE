#pragma once

#include <functional>

namespace odr
{

enum LogLevel
{
    Info = 0,
    Warn = 1,
    Error = 2
};

const char* log_level_to_string(const LogLevel level);

using LogFunction = std::function<void(const LogLevel level, const char* message)>;

void set_log_callback(LogFunction log_function);
void log_msg(const LogLevel level, const char* format, ...);

} // namespace odr