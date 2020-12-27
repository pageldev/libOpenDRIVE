SHELL = /bin/sh

CPP_FILES = \
	OpenDriveMap.cpp \
	Lanes.cpp \
	RefLine.cpp \
	Road.cpp \
	Geometries/Arc.cpp \
	Geometries/CubicSpline.cpp \
	Geometries/Line.cpp \
	Geometries/ParamPoly3.cpp \
	Geometries/RoadGeometry.cpp \
	Geometries/Spiral.cpp \
	Geometries/Spiral/odrSpiral.cpp \
	Thirdparty/pugixml/pugixml.cpp \
	Thirdparty/json11/json11.cpp

OBJ_FILES = $(CPP_FILES:%.cpp=$(BUILD_DIR)/%.o)
INCLUDE_DIRS = -I./ -I./Thirdparty

CFLAGS = -std=c++14 -O3 -Wall $(INCLUDE_DIRS)

.SECONDEXPANSION:
x64: BUILD_DIR = build/x64
x64: CC = g++
x64: dir $$(OBJ_FILES)
	$(CC) $(CFLAGS) -shared -o $(BUILD_DIR)/libOpenDrive.so $(OBJ_FILES)
	$(CC) $(CFLAGS) -L$(BUILD_DIR) -lOpenDrive -o $(BUILD_DIR)/main main.cpp

wasm: BUILD_DIR = build/wasm
wasm: CC = emcc
wasm: WASMFLAGS = --bind \
		-s ENVIRONMENT=web \
		-s MODULARIZE=1 \
		-s 'EXPORT_NAME="libOpenDrive"' \
		-s EXTRA_EXPORTED_RUNTIME_METHODS='["cwrap"]' \
		-s FORCE_FILESYSTEM=1 \
		-s ALLOW_MEMORY_GROWTH=1
wasm: dir $$(OBJ_FILES)
	$(CC) $(CFLAGS) $(WASMFLAGS) -o $(BUILD_DIR)/libOpenDrive.js $(OBJ_FILES)
	cp $(BUILD_DIR)/libOpenDrive.* Visualizer/

%.o: OBJ_FILE = $@
%.o: CPP_FILE = $(OBJ_FILE:$(BUILD_DIR)/%.o=%.cpp)
%.o: $(CPP_FILE)
	$(CC) $(CFLAGS) -c -fPIC -o $(OBJ_FILE) $(CPP_FILE)

dir: DIRS = $(dir $(OBJ_FILES))
dir:
	mkdir -p $(DIRS)

.PHONY: dir

clean:
	rm -rf build
	rm -f Visualizer/libOpenDrive*

.PHONY: clean