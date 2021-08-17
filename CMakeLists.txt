cmake_minimum_required(VERSION 3.10)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(libOpenDrive VERSION 0.1 DESCRIPTION ".xodr library")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

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
    OpenDriveMap.cpp
    RefLine.cpp
    Road.cpp
    RoadMark.cpp
    RoadNetworkMesh.cpp
    Thirdparty/json11/json11.cpp
    Thirdparty/pugixml/pugixml.cpp
)
target_include_directories(OpenDrive PRIVATE .)
target_include_directories(OpenDrive PRIVATE Thirdparty)

add_executable(test-xodr test.cpp)
target_link_libraries(test-xodr OpenDrive)
target_include_directories(test-xodr PUBLIC .)
