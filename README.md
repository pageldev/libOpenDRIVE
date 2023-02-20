# libOpenDRIVE
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
```

## Build
To build the library simply run:
```bash
mkdir build && cd build
cmake ..
make
```

This also builds an executable to test the library:
```bash
./build/test-xodr test.xodr
```
