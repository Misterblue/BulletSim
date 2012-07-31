# Build BulletSim

IDIR = /usr/local/include/bullet
LDIR = /usr/local/lib

# the Bullet libraries are linked statically so we don't have to also distribute the shared binaries
BULLETLIBS = $(LDIR)/libBulletDynamics.a $(LDIR)/libBulletCollision.a $(LDIR)/libLinearMath.a
 
#CC = gcc
CC = g++
CFLAGS = -I$(IDIR) -fPIC -g

BASEFILES = API.cpp API2.cpp BulletSim.cpp

OBJECTFILES = IPhysObject.cpp AvatarObject.cpp PrimObject.cpp GroundPlaneObject.cpp TerrainObject.cpp

COLLECTIONFILES = ObjectCollection.cpp

SRC = $(BASEFILES) $(OBJECTFILES) $(THINGFILE) $(COLLECTIONFILES)
# SRC = $(wildcard *.cpp)

BIN = $(patsubst %.cpp, %.o, $(SRC))


all: libBulletSim.so

libBulletSim.so : $(BIN)
	$(CC) -shared -Wl,-soname,libBulletSim.so -o libBulletSim.so $(BIN) $(BULLETLIBS) -lc

%.o: %.cpp
	$(CC) $(CFLAGS) -c $?

BulletSim.cpp : BulletSim.h GroundPlaneObject.h TerrainObject.h

BulletSim.h: ArchStuff.h APIData.h BSLogger.h IPhysObject.h TerrainObject.h ObjectCollection.h WorldData.h

API.cpp : BulletSim.h

API2.cpp : BulletSim.h

IPhysObject.cpp: IPhysObject.h AvatarObject.h PrimObject.h

IPhysObject.h: APIData.h WorldData.h

AvatarObject.cpp:  AvatarObject.h BulletSim.h

AvatarObject.h: IPhysObject.h APIData.h WorldData.h

PrimObject.cpp: PrimObject.h

PrimObject.h: IPhysObject.h APIData.h WorldData.h

GroundPlaneObject.cpp: GroundPlaneObject.h 

GroundPlaneObject.h: IPhysObject.h APIData.h WorldData.h

TerrainObject.cpp: TerrainObject.h

TerrainObject.h: IPhysObject.h APIData.h WorldData.h

ObjectCollection.cpp: ObjectCollection.h

ObjectCollection.h: IPhysObject.h ArchStuff.h

clean:
	rm -f *.o *.so
