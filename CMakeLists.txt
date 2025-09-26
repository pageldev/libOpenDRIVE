cmake_minimum_required(VERSION 3.14)

project(libOpenDrive VERSION 0.6.0 DESCRIPTION ".xodr library")

include(FetchContent)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(OPENDRIVE_BUILD_TESTS "Build OpenDrive tests" OFF)
option(BUILD_PYTHON_BINDINGS "Build Python bindings for libOpenDRIVE" OFF)

# Try to use an installed pugixml first; otherwise fetch it
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

# Build libOpenDRIVE
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
    target_compile_options(OpenDrive PRIVATE -Wall -Wextra)
endif()

target_link_libraries(OpenDrive PUBLIC pugixml::pugixml)

target_include_directories(
    OpenDrive PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Tests (optional)
if(OPENDRIVE_BUILD_TESTS)
    include(CTest)

    # Prefer an installed Catch2, fallback to fetching it
    find_package(Catch2 QUIET CONFIG)
    if(NOT TARGET Catch2::Catch2WithMain)
        FetchContent_Declare(
            Catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG v3.4.0
        )
        FetchContent_MakeAvailable(Catch2)
    endif()
    include(Catch)

    add_executable(tests tests/test.cpp)
    set_target_properties(tests PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
    )
    target_link_libraries(tests PRIVATE OpenDrive Catch2::Catch2WithMain)
    catch_discover_tests(tests WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests)
endif()

# Add Python bindings (optional, controlled by option)
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

    # Public include dir for the module (OpenDrive already exposes its include dir)
    target_include_directories(
        open_drive PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )

    # Install Python module
    install(TARGETS open_drive
        LIBRARY DESTINATION ${Python3_SITELIB}
    )
endif()

# Installation
install(
    TARGETS OpenDrive
    EXPORT OpenDriveTargets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
    EXPORT OpenDriveTargets
    NAMESPACE OpenDrive::
    FILE OpenDriveTargets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OpenDrive
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/OpenDriveConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY ExactVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/OpenDriveConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/OpenDriveConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OpenDrive
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/OpenDriveConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/OpenDriveConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OpenDrive
)