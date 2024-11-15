#include "Log.h"

#include <cstdarg>
#include <cstdio>
#include <string>

namespace odr
{

const char* log_level_to_string(const LogLevel level)
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

static LogFunction _log_callback = [](const LogLevel level, const char* message) { printf("[%s] %s\n", log_level_to_string(level), message); };

void set_log_callback(LogFunction log_function) { _log_callback = log_function; }

void log_msg(const LogLevel level, const char* format, ...)
{
    va_list args;
    va_start(args, format);
    const size_t size = std::vsnprintf(nullptr, 0, format, args);
    va_end(args);

    std::string buffer(size + 1, '\0');

    va_start(args, format);
    std::vsnprintf(&buffer[0], size + 1, format, args);
    va_end(args);

    _log_callback(level, buffer.c_str());
}

} // namespace odr