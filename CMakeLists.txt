cmake_minimum_required(VERSION 3.10)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(libOpenDrive VERSION 0.1.0 DESCRIPTION ".xodr library")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(./)
include_directories(Thirdparty)

add_library(OpenDrive SHARED
    Geometries/Arc.cpp
    Geometries/CubicSpline.cpp
    Geometries/Line.cpp
    Geometries/ParamPoly3.cpp
    Geometries/RoadGeometry.cpp
    Geometries/Spiral.cpp
    Geometries/Spiral/odrSpiral.cpp
    Lanes.cpp
    LaneSection.cpp
    Mesh.cpp
    OpenDriveMap.cpp
    RefLine.cpp
    Road.cpp
    RoadMark.cpp
    RoadObject.cpp
    RoutingGraph.cpp
    Thirdparty/pugixml/pugixml.cpp
)

if(EMSCRIPTEN)
    set(CMAKE_EXECUTABLE_SUFFIX ".js")
    add_executable(ModuleOpenDrive
        Embind.cpp
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