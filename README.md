# libOpenDRIVE
libOpenDRIVE is a lightweight, fast C++ library providing OpenDRIVE file parsing and 3D model generation. 

It's dependency-free, small and includes a [web-based viewer](https://sebastian-pagel.net/odrviewer/). It can be easily integrated in other projects and can be compiled to a WebAssembly library. A core function is the parsing of OpenDRIVE files and the generation of 3D models. The library targets OpenDRIVE version 1.4.


## Example
Here's an example of how code using libOpenDRIVE looks; it opens an xodr file, iterates over the elements of the road network and generates lanes meshes.

```c++
#include "OpenDriveMap.h"
#include "Lanes.h"
#include "Road.h"

#include <memory>
#include <stdio.h>

int main(void)
{
    odr::OpenDriveMap odr("data.xodr");
    for (std::shared_ptr<odr::Road> road : odr.get_roads())
    {
        printf("road: %s, length: %.2f\n", road->id.c_str(), road->length);
        for (std::shared_ptr<odr::LaneSection> lanesec : road->get_lanesections())
        {
            for (std::shared_ptr<odr::Lane> lane : lanesec->get_lanes())
            {
                auto lane_mesh = lane->get_mesh(lanesec->s0, lanesec->get_end(), 0.1);
            }
        }
    }
    return 0;
}
```


## Viewer
To use the included viewer first build the WebAssembly library and then run a webserver in the _Viewer/_ directory (e.g. `python3 -m http.server`). Or you can test the [viewer online](https://sebastian-pagel.net/odrviewer/).

![viewer-demo](https://user-images.githubusercontent.com/42587026/129762731-3c89900b-979e-436a-9a55-4c8745baa945.png)


## Build
To build the library simply run:
```bash
mkdir build && cd build
cmake ..
make
```

This also builds an executable to test the library:
```bash
./build/test-xodr Viewer/data.xodr
```


## WebAssembly
Install [emsdk](https://github.com/emscripten-core/emsdk) and run the following commands to build the WebAssembly library:

```bash
mkdir build && cd build
emcmake cmake ..
emmake make

cp ModuleOpenDrive.* ../Viewer
```

### Javascript Example

```js
fetch("./data.xodr").then((file_data) => {
    file_data.text().then((file_text) => {
        odr_map_config = {
            with_lateralProfile : PARAMS.lateralProfile,
            with_laneHeight : PARAMS.laneHeight,
            with_road_objects : false,
            center_map : true,
            abs_z_for_for_local_road_obj_outline : true
        };
        ModuleOpenDrive['FS_createDataFile'](".", "data.xodr", file_text, true, true);
        OpenDriveMap = new ModuleOpenDrive.OpenDriveMap("./data.xodr", odr_map_config);
    });
});
```
