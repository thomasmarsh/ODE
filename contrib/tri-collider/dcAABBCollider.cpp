#include "array.h"
#include "dxTriList.h"

#include "Opcode.h"
using namespace Opcode;

#include "dcAABBCollider.h"

//! Use CPU comparisons (comment that line to use standard FPU compares)
#define CPU_COMPARE

__forceinline bool dcAABBTreeCollider::BoxBoxOverlap(const Point& Extents0, const Point& Center0){
	/* Setup */
	const Point& Center1 = Box.mCenter;
	const Point& Extents1 = Box.mExtents;

	/* Originally by Pierre */
	float tx = Center0.x - Center1.x;
	float ex = Extents1.x + Extents0.x;
	if (AIR(tx) > IR(ex)) return false;

	float ty = Center0.y - Center1.y;
	float ey = Extents1.y + Extents0.y;
	if (AIR(ty) > IR(ey)) return false;

	float tz = Center0.z - Center1.z;
	float ez = Extents1.z + Extents0.z;
	if (AIR(tz) > IR(ez)) return false;

	return true;
}

//! TO BE DOCUMENTED
__forceinline bool planeBoxOverlap(const Point& normal, const float d, const Point& maxbox){
	Point vmin, vmax;
	for (udword q = 0; q <= 2; q++){
		if (normal[q] > 0.0f){
			vmin[q] = -maxbox[q];
			vmax[q] = maxbox[q];
		}
		else {
			vmin[q] = maxbox[q];
			vmax[q] =- maxbox[q];
		}
	}
	if ((normal | vmin) + d > 0.0f) return false;
	if ((normal | vmax) + d > 0.0f) return true;

	return false;
}

//! TO BE DOCUMENTED
#define AXISTEST_X01(a, b, fa, fb)							\
	min = a*v0.y - b*v0.z;									\
	max = a*v2.y - b*v2.z;									\
	if(min>max) {const float tmp=max; max=min; min=tmp;	}	\
	rad = fa * extents.y + fb * extents.z;					\
	if(min>rad || max<-rad) return false;

//! TO BE DOCUMENTED
#define AXISTEST_X2(a, b, fa, fb)							\
	min = a*v0.y - b*v0.z;									\
	max = a*v1.y - b*v1.z;									\
	if(min>max) {const float tmp=max; max=min; min=tmp;	}	\
	rad = fa * extents.y + fb * extents.z;					\
	if(min>rad || max<-rad) return false;

//! TO BE DOCUMENTED
#define AXISTEST_Y02(a, b, fa, fb)							\
	min = b*v0.z - a*v0.x;									\
	max = b*v2.z - a*v2.x;									\
	if(min>max) {const float tmp=max; max=min; min=tmp;	}	\
	rad = fa * extents.x + fb * extents.z;					\
	if(min>rad || max<-rad) return false;

//! TO BE DOCUMENTED
#define AXISTEST_Y1(a, b, fa, fb)							\
	min = b*v0.z - a*v0.x;									\
	max = b*v1.z - a*v1.x;									\
	if(min>max) {const float tmp=max; max=min; min=tmp;	}	\
	rad = fa * extents.x + fb * extents.z;					\
	if(min>rad || max<-rad) return false;

//! TO BE DOCUMENTED
#define AXISTEST_Z12(a, b, fa, fb)							\
	min = a*v1.x - b*v1.y;									\
	max = a*v2.x - b*v2.y;									\
	if(min>max) {const float tmp=max; max=min; min=tmp;	}	\
	rad = fa * extents.x + fb * extents.y;					\
	if(min>rad || max<-rad) return false;

//! TO BE DOCUMENTED
#define AXISTEST_Z0(a, b, fa, fb)							\
	min = a*v0.x - b*v0.y;									\
	max = a*v1.x - b*v1.y;									\
	if(min>max) {const float tmp=max; max=min; min=tmp;	}	\
	rad = fa * extents.x + fb * extents.y;					\
	if(min>rad || max<-rad) return false;

#define FINDMINMAX(x0, x1, x2, min, max)	\
	min = max = x0;							\
	if(x1<min) min=x1;						\
	if(x1>max) max=x1;						\
	if(x2<min) min=x2;						\
	if(x2>max) max=x2;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Triangle-Box overlap test using the separating axis theorem.
 *	This is the code from Tomas Möller, a bit optimized:
 *	- with some more lazy evaluation (faster path on PC)
 *	- with a tiny bit of assembly
 *	- with "SAT-lite" applied if needed
 *	- and perhaps with some more minor modifs...
 *
 *	\param		center		[in] box center
 *	\param		extents		[in] box extents
 *	\return		true if triangle & box overlap
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
__forceinline bool dcAABBTreeCollider::TriBoxOverlap(){
	// Setup

	const Point& center = Box.mCenter;
	const Point& extents = Box.mExtents;

	// use separating axis theorem to test overlap between triangle and box 
	// need to test for overlap in these directions: 
	// 1) the {x,y,z}-directions (actually, since we use the AABB of the triangle 
	//    we do not even need to test these) 
	// 2) normal of the triangle 
	// 3) crossproduct(edge from tri, {x,y,z}-directin) 
	//    this gives 3x3=9 more tests 

	// move everything so that the boxcenter is in (0,0,0) 
	Point v0, v1, v2;
	v0.x = LeafVerts[0]->x - center.x;
	v1.x = LeafVerts[1]->x - center.x;
	v2.x = LeafVerts[2]->x - center.x;

	float min,max;
	// Find min, max of the triangle in x-direction, and test for overlap in X
	FINDMINMAX(v0.x, v1.x, v2.x, min, max);
	if(min>extents.x || max<-extents.x) return false;

	// Same for Y
	v0.y = LeafVerts[0]->y - center.y;
	v1.y = LeafVerts[1]->y - center.y;
	v2.y = LeafVerts[2]->y - center.y;

	FINDMINMAX(v0.y, v1.y, v2.y, min, max);
	if(min>extents.y || max<-extents.y) return false;

	// Same for Z
	v0.z = LeafVerts[0]->z - center.z;
	v1.z = LeafVerts[1]->z - center.z;
	v2.z = LeafVerts[2]->z - center.z;

	FINDMINMAX(v0.z, v1.z, v2.z, min, max);
	if(min>extents.z || max<-extents.z) return false;

	// 2) Test if the box intersects the plane of the triangle
	// compute plane equation of triangle: normal*x+d=0
	// ### could be precomputed since we use the same leaf triangle several times
	const Point e0 = v1 - v0;
	const Point e1 = v2 - v1;
	const Point normal = e0 ^ e1;
	const float d = -normal|v0;
	if(!planeBoxOverlap(normal, d, extents)) return false;

	// 3) "Class III" tests
	float rad;
	// compute triangle edges
	// - edges lazy evaluated to take advantage of early exits
	// - fabs precomputed (half less work, possible since extents are always >0)
	// - customized macros to take advantage of the null component
	// - axis vector discarded, possibly saves useless movs
	
	const float fey0 = fabsf(e0.y);
	const float fez0 = fabsf(e0.z);
	AXISTEST_X01(e0.z, e0.y, fez0, fey0);
	const float fex0 = fabsf(e0.x);
	AXISTEST_Y02(e0.z, e0.x, fez0, fex0);
	AXISTEST_Z12(e0.y, e0.x, fey0, fex0);
	
	const float fey1 = fabsf(e1.y);
	const float fez1 = fabsf(e1.z);
	AXISTEST_X01(e1.z, e1.y, fez1, fey1);
	const float fex1 = fabsf(e1.x);
	AXISTEST_Y02(e1.z, e1.x, fez1, fex1);
	AXISTEST_Z0(e1.y, e1.x, fey1, fex1);
	
	const Point e2 = (*LeafVerts[0]) - (*LeafVerts[2]);
	const float fey2 = fabsf(e2.y);
	const float fez2 = fabsf(e2.z);
	AXISTEST_X2(e2.z, e2.y, fez2, fey2);
	const float fex2 = fabsf(e2.x);
	AXISTEST_Y1(e2.z, e2.x, fez2, fex2);
	AXISTEST_Z12(e2.y, e2.x, fey2, fex2);
	
	return true;
}

dcAABBTreeCollider::dcAABBTreeCollider(const dcVector3*& _Vertices, const int*& _Indices) : Vertices(_Vertices), Indices(_Indices){
	TCData = null;
}

dcAABBTreeCollider::~dcAABBTreeCollider(){
	//
}

#define FETCH_LEAF(TriIndex)										\
	LeafIndex = TriIndex;											\
	LeafVerts[0] = (Point*)&Vertices[Indices[TriIndex * 3 + 0]];	\
	LeafVerts[1] = (Point*)&Vertices[Indices[TriIndex * 3 + 1]];	\
	LeafVerts[2] = (Point*)&Vertices[Indices[TriIndex * 3 + 2]];


void dcAABBTreeCollider::InitQuery(const CollisionAABB& Box){
	Contacts.setSize(0);

	this->Box = Box;
}

/* Collide */
void dcAABBTreeCollider::Collide(const AABBNoLeafTree* Tree, const CollisionAABB& Box){
	InitQuery(Box);
	if (TCData != null){
		for (int i = 0; i < TCData->size(); i++){
			_Collide(TCData->operator[](i));
		}
	}
	else _Collide(Tree->GetNodes());
}


void dcAABBTreeCollider::_CollideTriBox(){
	if (TriBoxOverlap()){
		Contacts.push(LeafIndex);
	}
}

void dcAABBTreeCollider::_Collide(const AABBNoLeafNode* a){
	if (!BoxBoxOverlap(a->mAABB.mExtents, a->mAABB.mCenter)) return;

	if (a->HasLeaf()){
		FETCH_LEAF(a->GetPrimitive());
		_CollideTriBox();
	}
	else _Collide(a->GetPos());

	if (a->HasLeaf2()){
		FETCH_LEAF(a->GetPrimitive2());
		_CollideTriBox();
	}
	else _Collide(a->GetNeg());
}

/* GenerateTC */
void dcAABBTreeCollider::GenerateTC(const AABBNoLeafTree* Tree, const CollisionAABB& Box){
	if (TCData != null){
		InitQuery(Box);
		_GenerateTC(Tree->GetNodes());
	}
}

void dcAABBTreeCollider::_GenerateTC(const AABBNoLeafNode* a){
	if (!BoxBoxOverlap(a->mAABB.mExtents, a->mAABB.mCenter)) return;

	if (a->HasLeaf()){
		FETCH_LEAF(a->GetPrimitive());
		if (TriBoxOverlap()){
			TCData->push(a);
			return;
		}
	}
	else _GenerateTC(a->GetPos());

	if (a->HasLeaf2()){
		FETCH_LEAF(a->GetPrimitive2());
		if (TriBoxOverlap()){
			TCData->push(a);
			return;
		}
	}
	else _GenerateTC(a->GetNeg());
}