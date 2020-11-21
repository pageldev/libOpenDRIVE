CC = g++
CFLAGS = -std=c++14 -Wall $(INCLUDE_DIRS)
INCLUDE_DIRS = -I./ -I./$(THIRDPARTY_DIR)
THIRDPARTY_DIR = Thirdparty
BUILD_DIR = build
LIB_SUFFIX = so

CPP_FILES=$(shell find . -name '*.cpp' \
			-type f ! -name 'main.cpp' \
			-type f ! -path './$(BUILD_DIR)/*' \
			-type f ! -path './$(THIRDPARTY_DIR)/*')

OBJ_FILES=$(CPP_FILES:./%.cpp=./$(BUILD_DIR)/%.o)
DIRS=$(dir $(OBJ_FILES))

x86: CFLAGS += -O3
x86: lib main

wasm: CC = emcc
wasm: CFLAGS += -s ENVIRONMENT=web
wasm: WASMFLAGS += --bind -s MODULARIZE=1 -s 'EXPORT_NAME="libOpenDrive"' -s EXTRA_EXPORTED_RUNTIME_METHODS='["cwrap"]' -s FORCE_FILESYSTEM=1 -O3 -s ALLOW_MEMORY_GROWTH=1
wasm: BUILD_DIR = Visualizer/src
wasm: LIB_SUFFIX = js

lib: dir $(OBJ_FILES)
	$(CC) $(CFLAGS) $(WASMFLAGS) -shared -o $(BUILD_DIR)/libOpenDrive.$(LIB_SUFFIX) $(OBJ_FILES) ./Thirdparty/pugixml/pugixml.cpp ./Thirdparty/json11/json11.cpp

wasm:
	mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) $(WASMFLAGS) -o $(BUILD_DIR)/libOpenDrive.$(LIB_SUFFIX) $(CPP_FILES) ./Thirdparty/pugixml/pugixml.cpp ./Thirdparty/json11/json11.cpp

main: dir lib
	$(CC) $(CFLAGS) -L$(BUILD_DIR) -lOpenDrive -o $(BUILD_DIR)/main main.cpp

$(BUILD_DIR)/%.o: %.cpp
	$(CC) $(CFLAGS) -c -fPIC -o $@ $<

dir:
	mkdir -p $(DIRS)

clean:
	rm -rf $(BUILD_DIR)/*
	rm -rf Visualizer/src

.PHONY: clean
