#ifndef __DCTRILISTCOLLIDER_INCLUDED__
#define __DCTRILISTCOLLIDER_INCLUDED__

#define dSINGLE
#include <ode\\ode.h>

#include <Collision.h>

#include <Opcode.h>
using namespace Opcode;

#include "dcAABBCollider.h"
#include "dcOBBCollider.h"

#include "dTriList.h"
#include "dxTriList.h"

class dcTriListCollider{
	dxGeom* Geometry;	// The geom object
	dxTriList* GeomData;	// Buffered pointer to classdata

	struct CollisionTriangle{
		dcPlane Plane;			// The triangleplane
		Vertex ClosestPoint;	// The closest point to the other object
		Vertex* Vertices[3];	// The triangle's vertices
		dReal DistSq;			// The squared distance of the closest point to the other object

		dcPlane Edges[3];		// The planes that define the triangle's edges
	};
	dcVector<CollisionTriangle> CollisionTriangles;	// Internal buffer

	dContactGeom BoxContacts[3];	// Temporary buffer

	dcAABBTreeCollider AABBCollider;	// Opcode collider
	dcOBBTreeCollider OBBCollider;	// Opcode collider

	OPCODE_Model BVTree;

	dcVector<const AABBNoLeafNode*> TCData;	// TC buffer
public:
	dcTriListCollider(dxGeom* Geometry);
	~dcTriListCollider();

	dcArray<Vertex> Vertices;
	dcArray<dword> Indices;

	void Build();

	dword CollideBox(dxGeom* Box, dword Flags, dContactGeom* Contact, dword Stride);
	dword CollideSphere(dxGeom* Sphere, dword Flags, dContactGeom* Contact, dword Stride);
	bool GenerateTC(dxGeom* Object, const dcAABB& Box);
	void ClearTC();

	void Callback(dxGeom* Object, dcVector<dword>& TriIndices);
	bool Valid(dxGeom* Object, dword TriIndex);
};

#endif	//__DCTRILISTCOLLIDER_INCLUDED__