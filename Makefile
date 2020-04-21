CC = g++
CFLAGS = -std=c++11 $(INCLUDE_DIRS)
INCLUDE_DIRS = -I./ -I./Thirdparty
BUILD_DIR = build


x86: CFLAGS += -g -O3
x86: lib

wasm: CC = emcc
wasm: CFLAGS += -s MODULARIZE=1 -s 'EXPORT_NAME="libOpenDrive"' -s WASM=1 -s ENVIRONMENT=web -s EXTRA_EXPORTED_RUNTIME_METHODS='["cwrap"]'
wasm: lib


lib: dir odrSpiral Geometries Lanes Road OpenDriveMap Utils
	$(CC) $(CFLAGS) -shared -o $(BUILD_DIR)/libOpenDrive.so $(wildcard $(BUILD_DIR)/*.o) ./Thirdparty/pugixml/pugixml.cpp ./Thirdparty/json11/json11.cpp

main: dir lib
	$(CC) $(CFLAGS) -L$(BUILD_DIR) -lOpenDrive -o $(BUILD_DIR)/main main.cpp


Geometries: Geometries/RoadGeometry.cpp Line Arc Spiral ParamPoly3
	$(CC) $(CFLAGS) -c -o $(BUILD_DIR)/$@.o $<

Line: Geometries/Line.cpp
	$(CC) $(CFLAGS) -c -o $(BUILD_DIR)/$@.o $<

Arc: Geometries/Arc.cpp
	$(CC) $(CFLAGS) -c -o $(BUILD_DIR)/$@.o $<

Spiral: Geometries/Spiral.cpp odrSpiral
	$(CC) $(CFLAGS) -c -o $(BUILD_DIR)/$@.o $<

ParamPoly3: Geometries/ParamPoly3.cpp
	$(CC) $(CFLAGS) -c -o $(BUILD_DIR)/$@.o $<

odrSpiral: Geometries/Spiral/odrSpiral.c
	$(CC) -c -o $(BUILD_DIR)/$@.o $< 


Lanes: Lanes.cpp
	$(CC) $(CFLAGS) -c -o $(BUILD_DIR)/$@.o $<

Road: Road.cpp
	$(CC) $(CFLAGS) -c -o $(BUILD_DIR)/$@.o $<

OpenDriveMap: OpenDriveMap.cpp
	$(CC) $(CFLAGS) -c -o $(BUILD_DIR)/$@.o $<

Utils: Utils.cpp
	$(CC) $(CFLAGS) -c -o $(BUILD_DIR)/$@.o $<


dir:
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)/*

.PHONY: clean
