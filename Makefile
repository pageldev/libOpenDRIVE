.SUFFIXES:

CC = g++
XSD = xsd
CFLAGS = -std=c++14 -O3 -Wall $(INCLUDE_DIRS)
INCLUDE_DIRS = -I./ -I./Thirdparty
BUILD_DIR = build/x64

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
XSD_FILE = Thirdparty/odrSchema/OpenDRIVE_1.4H_Schema_Files.xsd
XSD_CXX = $(XSD_FILE:%.xsd=%.cxx)


.PHONY: all
all: $(BUILD_DIR)/libOpenDrive.so $(BUILD_DIR)/main

.PHONY: wasm
wasm: CC = emcc
wasm: WASMFLAGS = --bind \
		-s ENVIRONMENT=web \
		-s MODULARIZE=1 \
		-s 'EXPORT_NAME="libOpenDrive"' \
		-s EXTRA_EXPORTED_RUNTIME_METHODS='["cwrap"]' \
		-s FORCE_FILESYSTEM=1 \
		-s ALLOW_MEMORY_GROWTH=1
wasm: $(BUILD_DIR)/libOpenDrive.js
	cp $(BUILD_DIR)/libOpenDrive.* Visualizer/

$(BUILD_DIR)/libOpenDrive.so: $(OBJ_FILES) $(XSD_CXX)
	$(CC) $(CFLAGS) -shared -o $@ $(OBJ_FILES) OpenDRIVE_1.4H_Schema_Files.cxx

$(BUILD_DIR)/libOpenDrive.js: $(OBJ_FILES) $(XSD_CXX)
	$(CC) $(CFLAGS) $(WASMFLAGS) -o $@ $(OBJ_FILES) OpenDRIVE_1.4H_Schema_Files.cxx

$(XSD_CXX): $(XSD_FILE)
	$(XSD) cxx-tree --std c++11 --namespace-map '=odr' --generate-polymorphic --output-dir '$(dir $@)' $<

$(BUILD_DIR)/main: $(OBJ_FILES) main.cpp
	$(CC) $(CFLAGS) -o $@ $(OBJ_FILES) main.cpp

$(BUILD_DIR)/%.o: %.cpp
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c -fPIC -o $@ $<


.PHONY: clean
clean:
	rm -rf build
	rm -f $(dir $(XSD_CXX)){*.cxx,*.hxx}