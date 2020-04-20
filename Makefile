CC = g++
FLAGS = -std=c++11 -g -O3
BUILD_DIR = build

# main targets
lib: dir odrSpiral Geometries Lanes Road OpenDriveMap Utils
	$(CC) $(FLAGS) -shared `pkg-config --libs pugixml jsoncpp` -o $(BUILD_DIR)/libOpenDrive.so $(wildcard $(BUILD_DIR)/*.o)

main: dir lib
	$(CC) $(FLAGS) -L$(BUILD_DIR) -lOpenDrive -o $(BUILD_DIR)/main main.cpp

# geometries
odrSpiral: Spiral/odrSpiral.c
	$(CC) -c -o $(BUILD_DIR)/$@.o $< 

Geometries: Geometries/RoadGeometry.cpp Line Arc Spiral ParamPoly3
	$(CC) $(FLAGS) -c -o $(BUILD_DIR)/$@.o $<

Line: Geometries/Line.cpp
	$(CC) $(FLAGS) -c -o $(BUILD_DIR)/$@.o $<

Arc: Geometries/Arc.cpp
	$(CC) $(FLAGS) -c -o $(BUILD_DIR)/$@.o $<

Spiral: Geometries/Spiral.cpp odrSpiral
	$(CC) $(FLAGS) -I$(CURDIR) -c -o $(BUILD_DIR)/$@.o $<

ParamPoly3: Geometries/ParamPoly3.cpp
	$(CC) $(FLAGS) -c -o $(BUILD_DIR)/$@.o $<


Lanes: Lanes.cpp
	$(CC) $(FLAGS) -c -o $(BUILD_DIR)/$@.o $<

Road: Road.cpp
	$(CC) $(FLAGS) -c -o $(BUILD_DIR)/$@.o $<

OpenDriveMap: OpenDriveMap.cpp
	$(CC) $(FLAGS) `pkg-config --cflags pugixml jsoncpp` -c -o $(BUILD_DIR)/$@.o $<

Utils: Utils.cpp
	$(CC) $(FLAGS) -c -o $(BUILD_DIR)/$@.o $<


dir:
	mkdir -p $(BUILD_DIR)

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)/*
