CC = g++
CFLAGS = -std=c++11 $(INCLUDE_DIRS)
INCLUDE_DIRS = -I./ -I./$(THIRDPARTY_DIR)
THIRDPARTY_DIR = Thirdparty
BUILD_DIR = build
LIB_SUFFIX = so

CPP_FILES=$(shell find . -name '*.cpp' \
			-type f ! -name 'main.cpp' \
			-type f ! -path './$(BUILD_DIR)/*' \
			-type f ! -path './$(THIRDPARTY_DIR)/*')

x86: CFLAGS += -g -O3
x86: lib

wasm: CC = emcc
wasm: CFLAGS += -s ENVIRONMENT=web
wasm: WASMFLAGS += --bind -s MODULARIZE=1 -s 'EXPORT_NAME="libOpenDrive"' -s EXTRA_EXPORTED_RUNTIME_METHODS='["cwrap"]' -s FORCE_FILESYSTEM=1 -s EXPORT_ES6=1
wasm: BUILD_DIR = Visualizer/src
wasm: LIB_SUFFIX = js
wasm: lib

lib: dir
	$(CC) $(CFLAGS) $(WASMFLAGS) -shared -o $(BUILD_DIR)/libOpenDrive.$(LIB_SUFFIX) $(CPP_FILES) ./Thirdparty/pugixml/pugixml.cpp ./Thirdparty/json11/json11.cpp

main: dir lib
	$(CC) $(CFLAGS) -L$(BUILD_DIR) -lOpenDrive -o $(BUILD_DIR)/main main.cpp

dir:
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)/*

.PHONY: clean
