#include "dcTriListCollider.cpp"	// Allow inlining
#include "array.h"
#include "dxTriList.h"
#include "dcTriListCollider.h"

int dTriListClass = -1;

dcTriListCollider* GetData(dxGeom* TriList){
	dxTriList* Data = (dxTriList*)dGeomGetClassData(TriList);
	return Data->Collider;
}

inline bool ValidateCollision(dxGeom* o1, dxGeom* o2){
	dxBody* b1 = dGeomGetBody(o1);
	dxBody* b2 = dGeomGetBody(o2);

	if (b1){
		if (!dBodyIsEnabled(b1)){
			b1 = 0;
		}
	}
	if (b2){
		if (!dBodyIsEnabled(b2)){
			b2 = 0;
		}
	}
	return b1 || b2;
}

int dCollideSTL(dxGeom* TriList, dxGeom* Sphere, int Flags, dContactGeom* Contact, int Stride){
	if (ValidateCollision(Sphere, TriList)){
		return GetData(TriList)->CollideSphere(Sphere, Flags, Contact, Stride);
	}
	else return 0;
}

int dCollideBTL(dxGeom* TriList, dxGeom* Box, int Flags, dContactGeom* Contact, int Stride){
	if (ValidateCollision(Box, TriList)){
		return GetData(TriList)->CollideBox(Box, Flags, Contact, Stride);
	}
	else return 0;
}

int dAABBTestTL(dxGeom* TriList, dxGeom* Object, dReal AABB[6]){
	return 1;

	int Class = dGeomGetClass(Object);
	if (Class == dGeomGroupClass){
		CollisionAABB Box;
		Box.mCenter.x = (AABB[1] + AABB[0]) / 2.0f;
		Box.mCenter.y = (AABB[3] + AABB[2]) / 2.0f;
		Box.mCenter.z = (AABB[5] + AABB[4]) / 2.0f;
		Box.mExtents.x = (AABB[1] - AABB[0]) / 2.0f;
		Box.mExtents.y = (AABB[3] - AABB[2]) / 2.0f;
		Box.mExtents.z = (AABB[5] - AABB[4]) / 2.0f;
		return GetData(TriList)->GenerateTC(Object, Box);
	}
	else {
		GetData(TriList)->ClearTC();
		return 1;
	}
}

dColliderFn* dTriListColliderFn(int num){
	if (num == dBoxClass) return (dColliderFn*)&dCollideBTL;
	if (num == dSphereClass) return (dColliderFn*)&dCollideSTL;
	
	return 0;
}

void dDestroyTriList(dGeomID g){
	dxTriList* Data = (dxTriList*)dGeomGetClassData(g);
	delete Data->Collider;
}

/* External functions */
void dGeomTriListSetCallback(dGeomID g, dTriCallback* Callback){
	dxTriList* Data = (dxTriList*)dGeomGetClassData(g);
	Data->Callback = Callback;
}

dTriCallback* dGeomTriListGetCallback(dGeomID g){
	dxTriList* Data = (dxTriList*)dGeomGetClassData(g);
	return Data->Callback;
}

void dGeomTriListSetArrayCallback(dGeomID g, dTriArrayCallback* ArrayCallback){
	dxTriList* Data = (dxTriList*)dGeomGetClassData(g);
	Data->ArrayCallback = ArrayCallback;
}

dTriArrayCallback* dGeomTriListGetArrayCallback(dGeomID g){
	dxTriList* Data = (dxTriList*)dGeomGetClassData(g);
	return Data->ArrayCallback;
}

dxGeom* dCreateTriList(dSpaceID space, dTriCallback* Callback, dTriArrayCallback* ArrayCallback){
	if (dTriListClass == -1){
		dGeomClass c;
		c.bytes = sizeof(dxTriList);
		c.collider = &dTriListColliderFn;
		c.aabb = &dInfiniteAABB;
		c.aabb_test = &dAABBTestTL;
		c.dtor = &dDestroyTriList;

		dTriListClass = dCreateGeomClass(&c);
	}

	dxGeom* g = dCreateGeom(dTriListClass);
	if (space) dSpaceAdd(space, g);

	dxTriList* Data = (dxTriList*)dGeomGetClassData(g);
	Data->Callback = Callback;
	Data->ArrayCallback = ArrayCallback;
	Data->Collider = new dcTriListCollider(g);

	return g;
}

void dGeomTriListBuild(dGeomID g, const dcVector3* Vertices, int VertexCount, const int* Indices, int IndexCount){
	dxTriList* Data = (dxTriList*)dGeomGetClassData(g);
	Data->Collider->Vertices = Vertices;
	Data->Collider->Indices = Indices;
	Data->Collider->Build(VertexCount, IndexCount);
}

void dGeomTriListGetTriangle(dGeomID g, int Index, dVector3* v0, dVector3* v1, dVector3* v2){
	dxTriList* Data = (dxTriList*)dGeomGetClassData(g);
	if (v0 != 0){
		const dcVector3& v = Data->Collider->Vertices[Data->Collider->Indices[Index * 3 + 0]];
		(*v0)[0] = v[0];
		(*v0)[1] = v[1];
		(*v0)[2] = v[2];
		(*v0)[3] = REAL(0.0);
	}
	if (v1 != 0){
		const dcVector3& v = Data->Collider->Vertices[Data->Collider->Indices[Index * 3 + 1]];
		(*v1)[0] = v[0];
		(*v1)[1] = v[1];
		(*v1)[2] = v[2];
		(*v1)[3] = REAL(0.0);
	}
	if (v2 != 0){
		const dcVector3& v = Data->Collider->Vertices[Data->Collider->Indices[Index * 3 + 2]];
		(*v2)[0] = v[0];
		(*v2)[1] = v[1];
		(*v2)[2] = v[2];
		(*v2)[3] = REAL(0.0);
	}
}
