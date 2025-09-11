cmake_minimum_required(VERSION 3.14)

project(libOpenDrive VERSION 0.6.0 DESCRIPTION ".xodr library")

include(FetchContent)
include(GNUInstallDirs)

option(OPENDRIVE_BUILD_TESTS "Build OpenDrive tests" OFF)

find_package(pugixml QUIET CONFIG)
if(NOT TARGET pugixml::pugixml)
    if(TARGET pugixml)
        add_library(pugixml::pugixml ALIAS pugixml)
    else()
        FetchContent_Declare(
            pugixml
            GIT_REPOSITORY https://github.com/zeux/pugixml.git
            GIT_TAG v1.14
        )
        FetchContent_MakeAvailable(pugixml)
        add_library(pugixml::pugixml ALIAS pugixml)
    endif()
endif()

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
    src/Mesh.cpp
    src/OpenDriveMap.cpp
    src/RefLine.cpp
    src/Road.cpp
    src/RoadMark.cpp
    src/RoadNetworkMesh.cpp
    src/RoadObject.cpp
    src/RoadSignal.cpp
    src/RoutingGraph.cpp
)

add_library(OpenDrive ${SOURCES})
add_library(OpenDrive::OpenDrive ALIAS OpenDrive)
set_target_properties(OpenDrive PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)
if(MSVC)
    target_compile_options(OpenDrive PRIVATE /W3 /EHsc)
    target_compile_definitions(OpenDrive PRIVATE _USE_MATH_DEFINES)
else()
    target_compile_options(OpenDrive PRIVATE -Wall)
endif()

target_link_libraries(OpenDrive PUBLIC pugixml::pugixml)

target_include_directories(
    OpenDrive PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(OPENDRIVE_BUILD_TESTS)
    include(CTest)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.4.0
    )
    FetchContent_MakeAvailable(Catch2)
    include(Catch)

    add_executable(tests tests/test.cpp)
    target_link_libraries(tests PRIVATE OpenDrive Catch2::Catch2WithMain)
    catch_discover_tests(tests WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests)
endif()

install(
    TARGETS OpenDrive
    EXPORT OpenDriveTargets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(FILES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS OpenDrive EXPORT OpenDriveConfig)
install(EXPORT OpenDriveConfig NAMESPACE OpenDrive:: DESTINATION cmake)
