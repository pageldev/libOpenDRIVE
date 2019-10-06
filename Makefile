CC = g++
FLAGS = -std=c++11 -g
BUILD_DIR = build
CPP_FILES=$(shell find . -name '*.cpp' -type f ! -name 'main.cpp')
OBJ_FILES=$(CPP_FILES:.cpp=.o)

main: dir $(OBJ_FILES)
	$(CC) $(FLAGS) `pkg-config --cflags --libs pugixml jsoncpp` -o $(BUILD_DIR)/main $(wildcard $(BUILD_DIR)/*.o) main.cpp

dir:
	mkdir -p $(BUILD_DIR)

odrSpiral.o: Spiral/odrSpiral.c
	$(CC) -c -o $(BUILD_DIR)/$@ $< 

Geometries.o: Geometries.cpp odrSpiral.o
	$(CC) $(FLAGS) -c -o $(BUILD_DIR)/$@ $<

Lanes.o: Lanes.cpp
	$(CC) $(FLAGS) -c -o $(BUILD_DIR)/$@ $<

Road.o: Road.cpp
	$(CC) $(FLAGS) -c -o $(BUILD_DIR)/$@ $<

OpenDriveMap.o: OpenDriveMap.cpp
	$(CC) $(FLAGS) `pkg-config --cflags pugixml jsoncpp` -c -o $(BUILD_DIR)/$@ $<

Utils.o: Utils.cpp
	$(CC) $(FLAGS) -c -o $(BUILD_DIR)/$@ $<

%.o: %.cpp
	$(CC) $(FLAGS) -c -o $(BUILD_DIR)/$@ $<

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)/*
