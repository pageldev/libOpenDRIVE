CC = g++
CFLAGS = -std=c++14 -O3 -Wall $(INCLUDE_DIRS)
INCLUDE_DIRS = -I./ -I./$(THIRDPARTY_DIR)
THIRDPARTY_DIR = Thirdparty
BUILD_DIR = build

CPP_FILES=$(shell find . -name '*.cpp' \
			-type f ! -name 'main.cpp' \
			-type f ! -path './$(BUILD_DIR)/*' \
			-type f ! -path './$(THIRDPARTY_DIR)/*')

OBJ_FILES=$(CPP_FILES:./%.cpp=./$(BUILD_DIR)/%.o)
DIRS=$(dir $(OBJ_FILES))


x64: dir $(OBJ_FILES)
	$(CC) $(CFLAGS) -shared -o $(BUILD_DIR)/libOpenDrive.so $(OBJ_FILES) ./Thirdparty/pugixml/pugixml.cpp ./Thirdparty/json11/json11.cpp

wasm: CC = emcc
wasm: WASMFLAGS = --bind -s ENVIRONMENT=web \
		-s MODULARIZE=1 \
		-s 'EXPORT_NAME="libOpenDrive"' \
		-s EXTRA_EXPORTED_RUNTIME_METHODS='["cwrap"]' \
		-s FORCE_FILESYSTEM=1 \
		-s ALLOW_MEMORY_GROWTH=1
wasm: dir $(OBJ_FILES)
	$(CC) $(CFLAGS) $(WASMFLAGS) -o $(BUILD_DIR)/libOpenDrive.js $(OBJ_FILES) ./Thirdparty/pugixml/pugixml.cpp ./Thirdparty/json11/json11.cpp
	cp $(BUILD_DIR)/libOpenDrive.* Visualizer/


main: dir lib
	$(CC) $(CFLAGS) -L$(BUILD_DIR) -lOpenDrive -o $(BUILD_DIR)/main main.cpp

$(BUILD_DIR)/%.o: %.cpp
	$(CC) $(CFLAGS) -c -fPIC -o $@ $<

dir:
	mkdir -p $(DIRS)

clean:
	rm -rf $(BUILD_DIR)/*
	rm -f Visualizer/libOpenDrive*

.PHONY: clean
