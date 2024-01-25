# libOpenDRIVE

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.7771708.svg)](https://doi.org/10.5281/zenodo.7771708)


libOpenDRIVE is a **lightweight, dependency-free, fast C++ library** providing OpenDRIVE file parsing and 3D model generation. 

It's small and can be easily integrated in other projects. A core function is the parsing of OpenDRIVE files and the generation of 3D models. The library targets OpenDRIVE version 1.4.

## Example
Here's an example of how code using libOpenDRIVE looks. For a more complete example refer to [test.cpp](test.cpp).

```c++
// load map
odr::OpenDriveMap odr_map("test.xodr");

// iterate roads
for (odr::Road road : odr_map.get_roads())
    std::cout << "road: " << road.id << " length: " << road.length << std::endl;

// get xyz point for road coordinates
odr::Road odr_road = odr_map.id_to_road.at("508");
odr::Vec3D pt_xyz = odr_road.get_xyz(2.1 /*s*/, 1.0 /*t*/, 0.0 /*h*/);

// access road network attributes
std::string lane_type = odr_road.get_lanesection(0.0).id_to_lane.at(-1).type;

// use routing graph
odr::RoutingGraph routing_graph = odr_map.get_routing_graph();
odr::LaneKey from("516" /*road id*/, 0.0 /*lane section s0*/, 1 /*lane id*/);
odr::LaneKey to("501", 0.0, -1);
std::vector<odr::LaneKey> path = routing_graph.shortest_path(from, to);

// get road network mesh
odr::RoadNetworkMesh road_network_mesh = odr_map.get_road_network_mesh(0.1 /*eps*/);
std::cout << road_network_mesh.get_mesh().get_obj() << std::endl;
```

## Build
To build a static library by default, simply run:
```bash
mkdir build && cd build
cmake ..
make
```

If requiring a shared library, use:
```bash
cmake .. -DBUILD_SHARED_LIBS=ON
```

The build also provides an executable to test the library:
```bash
./build/test-xodr test.xodr
```

## Viewer
Check out the viewer at [odrviewer.io](https://odrviewer.io) which uses this library. Use the [odrviewer project](https://github.com/pageldev/odrviewer) to post issues and feature requests for odrviewer.io.

<sub>Info: The Viewer and WebAssembly bindings are no longer part of this project. This is to focus more on the library functionality and avoid having to keep the bindings up-to-date. Use [v0.3.0](https://github.com/pageldev/libOpenDRIVE/releases/tag/0.3.0) to get the last version that still includes Viewer and WebAssembly bindings. </sub>
