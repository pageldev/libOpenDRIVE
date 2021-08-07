```bash
mkdir build && cd build
emcmake cmake ..
emmake make

emcc -std=c++14 -O3 -Wall libOpenDrive.a
    --bind \
    -s ENVIRONMENT=web \
    -s MODULARIZE=1 \
    -s 'EXPORT_NAME="libOpenDrive"' \
    -s EXPORTED_RUNTIME_METHODS='["cwrap"]' \
    -s FORCE_FILESYSTEM=1 \
    -s ALLOW_MEMORY_GROWTH=1 \
    -o libOpenDrive.js
```