cmake_minimum_required(VERSION 3.13)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(libOpenDrive VERSION 0.6.0 DESCRIPTION ".xodr library")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(MSVC)
    set(CMAKE_CXX_FLAGS "/EHsc /W3")
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS 1)
    set(CMAKE_CXX_FLAGS_RELEASE "/O2")
    add_compile_definitions(_USE_MATH_DEFINES)
else()
    set(CMAKE_CXX_FLAGS "-Wall")
    set(CMAKE_CXX_FLAGS_DEBUG "-g")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif()

include(CTest)
include(FetchContent)
include(GNUInstallDirs)

# Fetch pugixml
FetchContent_Declare(
    pugixml
    GIT_REPOSITORY https://github.com/zeux/pugixml.git
    GIT_TAG v1.14
)
FetchContent_MakeAvailable(pugixml)

# Fetch Catch2 for tests
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.4.0
)
FetchContent_MakeAvailable(Catch2)
include(Catch)

# Define libOpenDRIVE sources
set(SOURCES
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
    src/Log.cpp
    src/Mesh.cpp
    src/OpenDriveMap.cpp
    src/RefLine.cpp
    src/Road.cpp
    src/RoadMark.cpp
    src/RoadNetworkMesh.cpp
    src/RoadObject.cpp
    src/RoadSignal.cpp
    src/RoutingGraph.cpp
    ${pugixml_SOURCE_DIR}/src/pugixml.cpp
)

# Build libOpenDRIVE
add_library(OpenDrive ${SOURCES})
target_include_directories(
    OpenDrive PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    $<BUILD_INTERFACE:${pugixml_SOURCE_DIR}/src>
)

# Build tests
add_executable(tests tests/test.cpp)
target_link_libraries(tests PRIVATE OpenDrive Catch2::Catch2WithMain)
catch_discover_tests(tests WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests)

# Add Python bindings (optional, controlled by option)
option(BUILD_PYTHON_BINDINGS "Build Python bindings for libOpenDRIVE" OFF)
if(BUILD_PYTHON_BINDINGS)
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
    find_package(pybind11 CONFIG REQUIRED)

    # Define binding sources
    set(BINDING_SOURCES
        bindings/bindings.cpp
        bindings/opendrivemap.cpp
        bindings/junction.cpp
        bindings/lane.cpp
        bindings/lanesection.cpp
        bindings/road.cpp
        bindings/refline.cpp
        bindings/mesh.cpp
        bindings/routinggraph.cpp
        bindings/roadmark.cpp
        bindings/roadnetworkmesh.cpp
        bindings/roadobject.cpp
        bindings/lanevalidityrecord.cpp
        bindings/roadsignal.cpp
        bindings/math.cpp
        bindings/xml_node.cpp
        # Add other binding files as needed
    )

    # Create Python module
    pybind11_add_module(open_drive ${BINDING_SOURCES})
    target_link_libraries(open_drive PRIVATE OpenDrive)
    target_include_directories(
        open_drive PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${pugixml_SOURCE_DIR}/src>
    )

    # Install Python module
    install(TARGETS open_drive
        LIBRARY DESTINATION ${Python3_SITELIB}
    )
endif()

# Installation
install(
    TARGETS OpenDrive
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(
    FILES
    ${pugixml_SOURCE_DIR}/src/pugixml.hpp
    ${pugixml_SOURCE_DIR}/src/pugiconfig.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS OpenDrive EXPORT OpenDriveConfig)
install(EXPORT OpenDriveConfig NAMESPACE OpenDrive:: DESTINATION cmake)
