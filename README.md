# libOpenDRIVE
libOpenDRIVE is a lightweight, fast C++ library providing OpenDRIVE file parsing and 3D model generation. 

It's dependency-free, small and includes a [web-based viewer](http://sebastian-pagel.net). It can be easily integrated in other projects and can be compiled to a WebAssembly library. A core function is the parsing of OpenDRIVE files and the generation of 3D models. The library targets OpenDRIVE version 1.4.

To build the library simply run:
```bash
mkdir build && pushd build
cmake ..
make
```

This also builds an executable to test the library:
```bash
./build/test-xodr Viewer/data.xodr
```

## WebAssembly
To build the WebAssembly library run:
```bash
mkdir build && pushd build
emcmake cmake ..
emmake make
popd

emcc -std=c++14 -O3 -Wall -I . \
    build/libOpenDrive.a Embind.cpp \
    --bind \
    -s ENVIRONMENT=web \
    -s MODULARIZE=1 \
    -s 'EXPORT_NAME="libOpenDrive"' \
    -s EXPORTED_RUNTIME_METHODS='["cwrap"]' \
    -s FORCE_FILESYSTEM=1 \
    -s ALLOW_MEMORY_GROWTH=1 \
    -o Viewer/libOpenDrive.js
```

## Viewer
To use the included viewer first build the WebAssembly library and then run a webserver in the _Viewer/_ directory (e.g. `python3 -m http.server`). Or you can test the [viewer online](https://sebastian-pagel.net).

![viewer-demo](https://user-images.githubusercontent.com/42587026/129762731-3c89900b-979e-436a-9a55-4c8745baa945.png)
