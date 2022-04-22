cmake_minimum_required(VERSION 3.10)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(libOpenDrive VERSION 0.1.0 DESCRIPTION ".xodr library")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if(MSVC)
SET(CMAKE_CXX_FLAGS "/EHsc")
SET(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS 1)
else()
set(CMAKE_CXX_FLAGS_DEBUG "-g")
endif()

include_directories(include)
include_directories(Thirdparty)

add_library(OpenDrive SHARED
    src/Geometries/Arc.cpp
    src/Geometries/CubicSpline.cpp
    src/Geometries/Line.cpp
    src/Geometries/ParamPoly3.cpp
    src/Geometries/RoadGeometry.cpp
    src/Geometries/Spiral.cpp
    src/Geometries/Spiral/odrSpiral.cpp
    src/Junction.cpp
    src/Lane.cpp
    src/LaneSection.cpp
    src/Mesh.cpp
    src/OpenDriveMap.cpp
    src/RefLine.cpp
    src/Road.cpp
    src/RoadMark.cpp
    src/RoadObject.cpp
    src/RoutingGraph.cpp
    Thirdparty/pugixml/pugixml.cpp
)

if(EMSCRIPTEN)
    set(CMAKE_EXECUTABLE_SUFFIX ".js")
    add_executable(ModuleOpenDrive
        src/Embind.cpp
        Viewer/RoadNetworkMesh.cpp
        Viewer/ViewerUtils.cpp
    )
    target_include_directories(ModuleOpenDrive PUBLIC Viewer)
    target_link_libraries(ModuleOpenDrive OpenDrive)
    set_target_properties(ModuleOpenDrive PROPERTIES LINK_FLAGS "\
        --bind \
        -s ENVIRONMENT=web \
        -s MODULARIZE=1 \
        -s 'EXPORT_NAME=\"libOpenDrive\"' \
        -s EXPORTED_RUNTIME_METHODS='[\"cwrap\"]' \
        -s FORCE_FILESYSTEM=1 \
        -s ALLOW_MEMORY_GROWTH=1 \
    ")
endif()

if(NOT EMSCRIPTEN)
    add_executable(test-xodr test.cpp)
    target_link_libraries(test-xodr OpenDrive)
endif()
