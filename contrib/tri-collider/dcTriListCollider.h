#ifndef __DCTRILISTCOLLIDER_INCLUDED__
#define __DCTRILISTCOLLIDER_INCLUDED__

#include <Opcode.h>
using namespace Opcode;

#include "dcAABBCollider.h"
#include "dcOBBCollider.h"

class dcTriListCollider{
	dxGeom* Geometry;	// The geom object
	dxTriList* GeomData;	// Buffered pointer to classdata

	struct CollisionTriangle{
		dcPlane Plane;			// The triangleplane
		dcVector3 ClosestPoint;	// The closest point to the other object
		const dcVector3* Vertices[3];	// The triangle's vertices
		float DistSq;			// The squared distance of the closest point to the other object

		dcPlane Edges[3];		// The planes that define the triangle's edges
	};
	dArray<CollisionTriangle> CollisionTriangles;	// Internal buffer

	dContactGeom BoxContacts[3];	// Temporary buffer

	dcAABBTreeCollider AABBCollider;	// Opcode collider
	dcOBBTreeCollider OBBCollider;	// Opcode collider

	OPCODE_Model BVTree;

	dArray<const AABBNoLeafNode*> TCData;	// TC buffer
public:
	dcTriListCollider(dxGeom* Geometry);
	~dcTriListCollider();

	const dcVector3* Vertices;
	const int* Indices;

	void Build(int VertexCount, int IndexCount);

	int CollideBox(dxGeom* Box, int Flags, dContactGeom* Contact, int Stride);
	int CollideSphere(dxGeom* Sphere, int Flags, dContactGeom* Contact, int Stride);
	bool GenerateTC(dxGeom* Object, const CollisionAABB& Box);
	void ClearTC();

	void Callback(dxGeom* Object, dArray<int>& TriIndices);
	bool Valid(dxGeom* Object, int TriIndex);
};

#endif	//__DCTRILISTCOLLIDER_INCLUDED__