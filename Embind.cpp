#ifdef __EMSCRIPTEN__
#include "OpenDriveMap.h"
#include <emscripten/bind.h>

EMSCRIPTEN_BINDINGS(OpenDriveMap)
{
    emscripten::class_<OpenDriveMap>("OpenDriveMap")
        .constructor<std::string>()
        .function("dump_json", &OpenDriveMap::dump_json)
        .property("xodr_file", &OpenDriveMap::xodr_file);
}
#endif