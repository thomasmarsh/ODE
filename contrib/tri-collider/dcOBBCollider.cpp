#include "array.h"
#include "dxTriList.h"

#include "Opcode.h"
using namespace Opcode;

#include "dcOBBCollider.h"

__forceinline void TransformPoint(Point& dest, const Point* source, const Matrix3x3& rot, const Point& trans){
	dest.x = trans.x + source->x * rot.m[0][0] + source->y * rot.m[1][0] + source->z * rot.m[2][0];
	dest.y = trans.y + source->x * rot.m[0][1] + source->y * rot.m[1][1] + source->z * rot.m[2][1];
	dest.z = trans.z + source->x * rot.m[0][2] + source->y * rot.m[1][2] + source->z * rot.m[2][2];
}

//! Use CPU comparisons (comment that line to use standard FPU compares)
#define CPU_COMPARE

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	OBB-OBB overlap test using the separating axis theorem.
 *	- original code by Gomez / Gamasutra (similar to Gottschalk's one in RAPID)
 *	- optimized for AABB trees by computing the rotation matrix once (SOLID-fashion)
 *	- the fabs matrix is precomputed as well and epsilon-tweaked (RAPID-style, we found this almost mandatory)
 *	- Class III axes can be disabled... (SOLID & Intel fashion)
 *	- ...or enabled to perform some profiling
 *	- CPU comparisons used when appropriate
 *	- lazy evaluation sometimes saves some work in case of early exits (unlike SOLID)
 *
 *	\param		a			[in] extent from box A
 *	\param		Pa			[in] center from box A
 *	\param		b			[in] extent from box B
 *	\param		Pb			[in] center from box B
 *	\return		true if boxes overlap
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
__forceinline bool dcOBBTreeCollider::BoxBoxOverlap(const Point& a, const Point& Pa){
	const Point& b = Box;

	float t,t2;

	// Class I : A's basis vectors
#ifdef CPU_COMPARE
	float Tx = mT1to0.x - Pa.x;
	t = a.x + BBx1;
	if(AIR(Tx) > IR(t))	return false;

	float Ty = mT1to0.y - Pa.y;
	t = a.y + BBy1;
	if(AIR(Ty) > IR(t))	return false;

	float Tz = mT1to0.z - Pa.z;
	t = a.z + BBz1;
	if(AIR(Tz) > IR(t))	return false;
#else
	float Tx = mT1to0.x - Pa.x;
	t = a.x + BBx1;
	if(fabsf(Tx) > t)	return false;

	float Ty = mT1to0.y - Pa.y;
	t = a.y + BBy1;
	if(fabsf(Ty) > t)	return false;

	float Tz = mT1to0.z - Pa.z;
	t = a.z + BBz1;
	if(fabsf(Tz) > t)	return false;
#endif

	// Class II : B's basis vectors
#ifdef CPU_COMPARE
	t = Tx*mR1to0.m[0][0] + Ty*mR1to0.m[0][1] + Tz*mR1to0.m[0][2];	t2 = a.x*mAR.m[0][0] + a.y*mAR.m[0][1] + a.z*mAR.m[0][2] + b.x;
	if(AIR(t)>IR(t2))	return false;

	t = Tx*mR1to0.m[1][0] + Ty*mR1to0.m[1][1] + Tz*mR1to0.m[1][2];	t2 = a.x*mAR.m[1][0] + a.y*mAR.m[1][1] + a.z*mAR.m[1][2] + b.y;
	if(AIR(t)>IR(t2))	return false;

	t = Tx*mR1to0.m[2][0] + Ty*mR1to0.m[2][1] + Tz*mR1to0.m[2][2];	t2 = a.x*mAR.m[2][0] + a.y*mAR.m[2][1] + a.z*mAR.m[2][2] + b.z;
	if(AIR(t)>IR(t2))	return false;
#else
	t = Tx*mR1to0.m[0][0] + Ty*mR1to0.m[0][1] + Tz*mR1to0.m[0][2];	t2 = a.x*mAR.m[0][0] + a.y*mAR.m[0][1] + a.z*mAR.m[0][2] + b.x;
	if(fabsf(t) > t2)	return false;

	t = Tx*mR1to0.m[1][0] + Ty*mR1to0.m[1][1] + Tz*mR1to0.m[1][2];	t2 = a.x*mAR.m[1][0] + a.y*mAR.m[1][1] + a.z*mAR.m[1][2] + b.y;
	if(fabsf(t) > t2)	return false;

	t = Tx*mR1to0.m[2][0] + Ty*mR1to0.m[2][1] + Tz*mR1to0.m[2][2];	t2 = a.x*mAR.m[2][0] + a.y*mAR.m[2][1] + a.z*mAR.m[2][2] + b.z;
	if(fabsf(t) > t2)	return false;
#endif

	// Class III : 9 cross products
#ifdef CPU_COMPARE
	t = Tz*mR1to0.m[0][1] - Ty*mR1to0.m[0][2];	t2 = a.y*mAR.m[0][2] + a.z*mAR.m[0][1] + BB_1;	if(AIR(t) > IR(t2))	return false;	// L = A0 x B0
	t = Tz*mR1to0.m[1][1] - Ty*mR1to0.m[1][2];	t2 = a.y*mAR.m[1][2] + a.z*mAR.m[1][1] + BB_2;	if(AIR(t) > IR(t2))	return false;	// L = A0 x B1
	t = Tz*mR1to0.m[2][1] - Ty*mR1to0.m[2][2];	t2 = a.y*mAR.m[2][2] + a.z*mAR.m[2][1] + BB_3;	if(AIR(t) > IR(t2))	return false;	// L = A0 x B2
	t = Tx*mR1to0.m[0][2] - Tz*mR1to0.m[0][0];	t2 = a.x*mAR.m[0][2] + a.z*mAR.m[0][0] + BB_4;	if(AIR(t) > IR(t2))	return false;	// L = A1 x B0
	t = Tx*mR1to0.m[1][2] - Tz*mR1to0.m[1][0];	t2 = a.x*mAR.m[1][2] + a.z*mAR.m[1][0] + BB_5;	if(AIR(t) > IR(t2))	return false;	// L = A1 x B1
	t = Tx*mR1to0.m[2][2] - Tz*mR1to0.m[2][0];	t2 = a.x*mAR.m[2][2] + a.z*mAR.m[2][0] + BB_6;	if(AIR(t) > IR(t2))	return false;	// L = A1 x B2
	t = Ty*mR1to0.m[0][0] - Tx*mR1to0.m[0][1];	t2 = a.x*mAR.m[0][1] + a.y*mAR.m[0][0] + BB_7;	if(AIR(t) > IR(t2))	return false;	// L = A2 x B0
	t = Ty*mR1to0.m[1][0] - Tx*mR1to0.m[1][1];	t2 = a.x*mAR.m[1][1] + a.y*mAR.m[1][0] + BB_8;	if(AIR(t) > IR(t2))	return false;	// L = A2 x B1
	t = Ty*mR1to0.m[2][0] - Tx*mR1to0.m[2][1];	t2 = a.x*mAR.m[2][1] + a.y*mAR.m[2][0] + BB_9;	if(AIR(t) > IR(t2))	return false;	// L = A2 x B2
#else
	t = Tz*mR1to0.m[0][1] - Ty*mR1to0.m[0][2];	t2 = a.y*mAR.m[0][2] + a.z*mAR.m[0][1] + BB_1;	if(fabsf(t) > t2)	return false;
	t = Tz*mR1to0.m[1][1] - Ty*mR1to0.m[1][2];	t2 = a.y*mAR.m[1][2] + a.z*mAR.m[1][1] + BB_2;	if(fabsf(t) > t2)	return false;
	t = Tz*mR1to0.m[2][1] - Ty*mR1to0.m[2][2];	t2 = a.y*mAR.m[2][2] + a.z*mAR.m[2][1] + BB_3;	if(fabsf(t) > t2)	return false;
	t = Tx*mR1to0.m[0][2] - Tz*mR1to0.m[0][0];	t2 = a.x*mAR.m[0][2] + a.z*mAR.m[0][0] + BB_4;	if(fabsf(t) > t2)	return false;
	t = Tx*mR1to0.m[1][2] - Tz*mR1to0.m[1][0];	t2 = a.x*mAR.m[1][2] + a.z*mAR.m[1][0] + BB_5;	if(fabsf(t) > t2)	return false;
	t = Tx*mR1to0.m[2][2] - Tz*mR1to0.m[2][0];	t2 = a.x*mAR.m[2][2] + a.z*mAR.m[2][0] + BB_6;	if(fabsf(t) > t2)	return false;
	t = Ty*mR1to0.m[0][0] - Tx*mR1to0.m[0][1];	t2 = a.x*mAR.m[0][1] + a.y*mAR.m[0][0] + BB_7;	if(fabsf(t) > t2)	return false;
	t = Ty*mR1to0.m[1][0] - Tx*mR1to0.m[1][1];	t2 = a.x*mAR.m[1][1] + a.y*mAR.m[1][0] + BB_8;	if(fabsf(t) > t2)	return false;
	t = Ty*mR1to0.m[2][0] - Tx*mR1to0.m[2][1];	t2 = a.x*mAR.m[2][1] + a.y*mAR.m[2][0] + BB_9;	if(fabsf(t) > t2)	return false;
#endif
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
__forceinline bool dcOBBTreeCollider::TriBoxOverlap(){
	// Setup
	const Point& extents = Box;

	// use separating axis theorem to test overlap between triangle and box 
	// need to test for overlap in these directions: 
	// 1) the {x,y,z}-directions (actually, since we use the AABB of the triangle 
	//    we do not even need to test these) 
	// 2) normal of the triangle 
	// 3) crossproduct(edge from tri, {x,y,z}-directin) 
	//    this gives 3x3=9 more tests 

	// move everything so that the boxcenter is in (0,0,0) 
	const Point& v0 = LeafVerts[0];
	const Point& v1 = LeafVerts[1];
	const Point& v2 = LeafVerts[2];
	
	// First, test overlap in the {x,y,z}-directions

	// find min, max of the triangle in x-direction, and test for overlap in X

	float min,max;
	// Find min, max of the triangle in x-direction, and test for overlap in X
	FINDMINMAX(v0.x, v1.x, v2.x, min, max);
	if(min>extents.x || max<-extents.x) return false;

	FINDMINMAX(v0.y, v1.y, v2.y, min, max);
	if(min>extents.y || max<-extents.y) return false;

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
	
	const Point e2 = LeafVerts[0] - LeafVerts[2];
	const float fey2 = fabsf(e2.y);
	const float fez2 = fabsf(e2.z);
	AXISTEST_X2(e2.z, e2.y, fez2, fey2);
	const float fex2 = fabsf(e2.x);
	AXISTEST_Y1(e2.z, e2.x, fez2, fex2);
	AXISTEST_Z12(e2.y, e2.x, fey2, fex2);
	
	return true;
}

dcOBBTreeCollider::dcOBBTreeCollider(const dcVector3*& _Vertices, const int*& _Indices) : Vertices(_Vertices), Indices(_Indices){
	TCData = null;
}

dcOBBTreeCollider::~dcOBBTreeCollider(){
	//
}

void dcOBBTreeCollider::InitQuery(const Point& Box, const Matrix4x4& BoxMatrix){
	Contacts.setSize(0);

	this->Box = Box;

	Matrix4x4 InvBoxMatrix;
	InvertPRMatrix(InvBoxMatrix, BoxMatrix);

	/* Setup matrices */
	mR0to1 = InvBoxMatrix;	InvBoxMatrix.GetTrans(mT0to1);
	mR1to0 = BoxMatrix;		BoxMatrix.GetTrans(mT1to0);

	// Precompute absolute 1-to-0 rotation matrix
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			// Epsilon value prevents floating-point inaccuracies (strategy borrowed from RAPID)
			mAR.m[i][j] = 1e-6f + fabsf(mR1to0.m[i][j]);
		}
	}

	/* Precompute box-box data */
	BBx1 = Box.x*mAR.m[0][0] + Box.y*mAR.m[1][0] + Box.z*mAR.m[2][0];
	BBy1 = Box.x*mAR.m[0][1] + Box.y*mAR.m[1][1] + Box.z*mAR.m[2][1];
	BBz1 = Box.x*mAR.m[0][2] + Box.y*mAR.m[1][2] + Box.z*mAR.m[2][2];

	BB_1 = Box.y*mAR.m[2][0] + Box.z*mAR.m[1][0];
	BB_2 = Box.x*mAR.m[2][0] + Box.z*mAR.m[0][0];
	BB_3 = Box.x*mAR.m[1][0] + Box.y*mAR.m[0][0];
	BB_4 = Box.y*mAR.m[2][1] + Box.z*mAR.m[1][1];
	BB_5 = Box.x*mAR.m[2][1] + Box.z*mAR.m[0][1];
	BB_6 = Box.x*mAR.m[1][1] + Box.y*mAR.m[0][1];
	BB_7 = Box.y*mAR.m[2][2] + Box.z*mAR.m[1][2];
	BB_8 = Box.x*mAR.m[2][2] + Box.z*mAR.m[0][2];
	BB_9 = Box.x*mAR.m[1][2] + Box.y*mAR.m[0][2];
}

void dcOBBTreeCollider::Collide(const AABBNoLeafTree* Tree, const Point& Box, const Matrix4x4& BoxMatrix){
	InitQuery(Box, BoxMatrix);

	if (TCData != null){
		for (int i = 0; i < TCData->size(); i++){
			_Collide(TCData->operator[](i));
		}
	}
	else _Collide(Tree->GetNodes());
}

void dcOBBTreeCollider::_CollideTriBox(){
	if (TriBoxOverlap()){
		Contacts.push(LeafIndex);
	}
}

#define FETCH_LEAF(TriIndex, rot, trans)													\
	LeafIndex = TriIndex;																	\
	TransformPoint(LeafVerts[0], (Point*)&Vertices[Indices[TriIndex * 3 + 0]], rot, trans);	\
	TransformPoint(LeafVerts[1], (Point*)&Vertices[Indices[TriIndex * 3 + 1]], rot, trans);	\
	TransformPoint(LeafVerts[2], (Point*)&Vertices[Indices[TriIndex * 3 + 2]], rot, trans);

void dcOBBTreeCollider::_Collide(const AABBNoLeafNode* a){
	// Perform BV-BV overlap test
	if (!BoxBoxOverlap(a->mAABB.mExtents, a->mAABB.mCenter)) return;

	if (a->HasLeaf()){
		FETCH_LEAF(a->GetPrimitive(), mR0to1, mT0to1);
		_CollideTriBox();
	}
	else _Collide(a->GetPos());

	if (a->HasLeaf2()){
		FETCH_LEAF(a->GetPrimitive2(), mR0to1, mT0to1);
		_CollideTriBox();
	}
	else _Collide(a->GetNeg());
}
