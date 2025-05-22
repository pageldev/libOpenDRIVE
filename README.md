# libOpenDRIVE

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.7771708.svg)](https://doi.org/10.5281/zenodo.7771708)


libOpenDRIVE is a **lightweight, dependency-free, fast C++ library** providing OpenDRIVE file parsing and 3D model generation. 

It's small and can be easily integrated in other projects. A core function is the parsing of OpenDRIVE files and the generation of 3D models. The library targets OpenDRIVE version 1.4.

## Example
Here's an example of how code using libOpenDRIVE looks. For a more complete example refer to [tests/test.cpp](tests/test.cpp).

```c++
// load map
odr::OpenDriveMap odr_map("tests/test.xodr");

// iterate roads
for (odr::Road road : odr_map.get_roads())
    std::cout << "road: " << road.id << " length: " << road.length << std::endl;

// get xyz point for road coordinates
odr::Road odr_road = odr_map.get_road("17");
odr::Vec3D pt_xyz = odr_road.get_xyz(2.1 /*s*/, 1.0 /*t*/, 0.0 /*h*/);

// access road network attributes
std::string lane_type = odr_road.get_lanesection(0.0).get_lane(-1).type;

// use routing graph
odr::RoutingGraph routing_graph = odr_map.get_routing_graph();
odr::LaneKey from("17" /*road id*/, 0.0 /*lane section s0*/, 1 /*lane id*/);
odr::LaneKey to("41", 0.0, -1);
std::vector<odr::LaneKey> path = routing_graph.shortest_path(from, to);

// get road network mesh
odr::RoadNetworkMesh road_network_mesh = odr_map.get_road_network_mesh(0.1 /*eps*/);
std::cout << road_network_mesh.get_mesh().get_obj() << std::endl;
```

## Build
To build a static library by default, simply run:
```bash
mkdir build
cd build
cmake ..
make
```

If requiring a shared library, use:
```bash
cmake -DBUILD_SHARED_LIBS=ON ..
```

### Python Bindings with pybind11

To build the Python bindings using pybind11 and enable installation via pip in editable mode (`pip install -e .`), follow these steps:

1. **Create and enter the build directory:**

   ```bash
   mkdir build
   cd build

2. **Configure the build with Python bindings enabled, specifying your Python interpreter:**

   ```
   cmake -DBUILD_SHARED_LIBS=ON -DBUILD_PYTHON_BINDINGS=ON -DPython3_EXECUTABLE=$(which python) ..
   ```

3. **Build the project:**
   ```
   make
   ```

4. **Install the Python package in editable mode:**
   From the root directory of the repository (not inside `build`), run:

   ```
   pip install -e .
   ```
   This command uses the generated Python bindings and sets up the package in your environment so that changes to the source code reflect immediately without reinstalling.


## Viewer
Check out the viewer at [odrviewer.io](https://odrviewer.io) which uses this library. Use the [odrviewer project](https://github.com/pageldev/odrviewer) to post issues and feature requests for odrviewer.io.

<sub>Info: The Viewer and WebAssembly bindings are no longer part of this project. This is to focus more on the library functionality and avoid having to keep the bindings up-to-date. Use [v0.3.0](https://github.com/pageldev/libOpenDRIVE/releases/tag/0.3.0) to get the last version that still includes Viewer and WebAssembly bindings. </sub>
