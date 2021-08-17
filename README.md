
## Viewer
![viewer-demo](https://user-images.githubusercontent.com/42587026/129762731-3c89900b-979e-436a-9a55-4c8745baa945.png)

## WASM

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
