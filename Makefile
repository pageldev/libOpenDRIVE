CC = g++
FLAGS = -std=c++11 -g -O3
BUILD_DIR = build
# CPP_FILES=$(shell find . -name '*.cpp' -type f ! -name 'main.cpp')
# OBJ_FILES=$(CPP_FILES:.cpp=.o)

lib: dir odrSpiral Geometries Lanes Road OpenDriveMap Utils
	$(CC) $(FLAGS) -shared `pkg-config --libs pugixml jsoncpp` -o $(BUILD_DIR)/libOpenDrive.so $(wildcard $(BUILD_DIR)/*.o)

main: dir lib
	$(CC) $(FLAGS) -L$(BUILD_DIR) -lOpenDrive -o $(BUILD_DIR)/main main.cpp

dir:
	mkdir -p $(BUILD_DIR)

odrSpiral: Spiral/odrSpiral.c
	$(CC) -c -o $(BUILD_DIR)/$@.o $< 

Geometries: Geometries/Geometries.cpp odrSpiral
	$(CC) $(FLAGS) -I$(CURDIR) -c -o $(BUILD_DIR)/$@.o $<

Lanes: Lanes.cpp
	$(CC) $(FLAGS) -c -o $(BUILD_DIR)/$@.o $<

Road: Road.cpp
	$(CC) $(FLAGS) -c -o $(BUILD_DIR)/$@.o $<

OpenDriveMap: OpenDriveMap.cpp
	$(CC) $(FLAGS) `pkg-config --cflags pugixml jsoncpp` -c -o $(BUILD_DIR)/$@.o $<

Utils: Utils.cpp
	$(CC) $(FLAGS) -c -o $(BUILD_DIR)/$@.o $<

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)/*
