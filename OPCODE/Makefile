
# Makefile for building OPCODE in unix using gcc

SRC=\
	OPC_AABBCollider.cpp \
	OPC_AABBTree.cpp \
	OPC_BaseModel.cpp \
	OPC_BoxPruning.cpp \
	OPC_Collider.cpp \
	OPC_Common.cpp \
	OPC_HybridModel.cpp \
	OPC_LSSCollider.cpp \
	OPC_MeshInterface.cpp \
	OPC_Model.cpp \
	OPC_OBBCollider.cpp \
	Opcode.cpp \
	OPC_OptimizedTree.cpp \
	OPC_Picking.cpp \
	OPC_PlanesCollider.cpp \
	OPC_RayCollider.cpp \
	OPC_SphereCollider.cpp \
	OPC_SweepAndPrune.cpp \
	OPC_TreeBuilders.cpp \
	OPC_TreeCollider.cpp \
	OPC_VolumeCollider.cpp \
	Ice/IceAABB.cpp \
	Ice/IceContainer.cpp \
	Ice/IceHPoint.cpp \
	Ice/IceIndexedTriangle.cpp \
	Ice/IceMatrix3x3.cpp \
	Ice/IceMatrix4x4.cpp \
	Ice/IceOBB.cpp \
	Ice/IcePlane.cpp \
	Ice/IcePoint.cpp \
	Ice/IceRandom.cpp \
	Ice/IceRay.cpp \
	Ice/IceRevisitedRadix.cpp \
	Ice/IceSegment.cpp \
	Ice/IceTriangle.cpp \
	Ice/IceUtils.cpp

VPATH=Ice

OBJ=$(SRC:%.cpp=%.o)

all: libopcode.a

libopcode.a: $(OBJ)
	ar rcv $@ $(OBJ)

clean:
	-rm -f *.o Ice/*.o libopcode.a

# GCC versions 3.X generate bad code for some modules
# with -O2, so we use -O1 until we get this tied down a bit more.
# There are at least two problematic code modules with -O2, one of
# which is OPC_TreeCollider (if you identify the other, which causes
# problems with trimesh-trimesh collisions, then let us know).
%.o : %.cpp
	g++ -c -o $@ -g -O1 -I. -fno-rtti -fno-exceptions $<

depend:
	g++ -MM -I. $(SRC) > Dependencies

include Dependencies
