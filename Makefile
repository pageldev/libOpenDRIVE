CC = g++
FLAGS = -std=c++11 -g `pkg-config --cflags --libs pugixml`
BUILDDIR = build

all: dir OpenDriveMap.o
	$(CC) $(FLAGS) -o $(BUILDDIR)/main main.cpp $(BUILDDIR)/OpenDriveMap.o 

dir:
	mkdir -p $(BUILDDIR)

OpenDriveMap.o:
	$(CC) $(FLAGS) -c -o $(BUILDDIR)/OpenDriveMap.o OpenDriveMap.cpp

clean:
	rm -rf $(BUILDDIR)/*
