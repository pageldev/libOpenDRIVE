CC = g++
FLAGS = -std=c++11 -g `pkg-config --cflags --libs pugixml`
BUILD_DIR = build
CPP_FILES=$(shell find . -name '*.cpp' -type f ! -name 'main.cpp')
OBJ_FILES=$(CPP_FILES:.cpp=.o)

all: dir $(OBJ_FILES)
	$(CC) $(FLAGS) -o $(BUILD_DIR)/main $(wildcard $(BUILD_DIR)/*.o) main.cpp

dir:
	mkdir -p $(BUILD_DIR)

Geometries.o: Geometries.cpp odrSpiral.o
	$(CC) $(FLAGS) -c -o $(BUILD_DIR)/$@ $<

odrSpiral.o: Spiral/odrSpiral.c
	$(CC) -c -o $(BUILD_DIR)/$@ $< 

%.o: %.cpp
	$(CC) $(FLAGS) -c -o $(BUILD_DIR)/$@ $<

clean:
	rm -rf $(BUILD_DIR)/*
