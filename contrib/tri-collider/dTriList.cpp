#include <dcCore.h>

#include "dcTriListCollider.cpp"	// Allow inlining

int dTriListClass = -1;

dcTriListCollider* GetData(dxGeom* TriList){
	dxTriList* Data = (dxTriList*)dGeomGetClassData(TriList);
	return Data->Collider;
}

int dCollideSTL(dxGeom* TriList, dxGeom* Sphere, int Flags, dContactGeom* Contact, int Stride){
	return GetData(TriList)->CollideSphere(Sphere, Flags, Contact, Stride);
}

int dCollideBTL(dxGeom* TriList, dxGeom* Box, int Flags, dContactGeom* Contact, int Stride){
	return GetData(TriList)->CollideBox(Box, Flags, Contact, Stride);
}

int dAABBTestTL(dxGeom* TriList, dxGeom* Object, dReal AABB[6]){
	dword Class = dGeomGetClass(Object);
	if (Class == dGeomGroupClass){
		dcAABB Box;
		Box.Center.x = (AABB[1] + AABB[0]) / 2.0f;
		Box.Center.y = (AABB[3] + AABB[2]) / 2.0f;
		Box.Center.z = (AABB[5] + AABB[4]) / 2.0f;
		Box.Extents.x = (AABB[1] - AABB[0]) / 2.0f;
		Box.Extents.y = (AABB[3] - AABB[2]) / 2.0f;
		Box.Extents.z = (AABB[5] - AABB[4]) / 2.0f;
		return GetData(TriList)->GenerateTC(Object, Box);
	}
	else {
		GetData(TriList)->ClearTC();
		return 1;
	}
}

dColliderFn* dTriListColliderFn(int num){ 
	if (num == dSphereClass) return (dColliderFn*)&dCollideSTL;
	if (num == dBoxClass) return (dColliderFn*)&dCollideBTL;
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

dcArray<Vertex>& dGeomTriListGetVertexArray(dGeomID g){
	dxTriList* Data = (dxTriList*)dGeomGetClassData(g);
	return Data->Collider->Vertices;
}

dcArray<dword>& dGeomTriListGetIndexArray(dGeomID g){
	dxTriList* Data = (dxTriList*)dGeomGetClassData(g);
	return Data->Collider->Indices;
}

void dGeomTriListBuild(dGeomID g){
	dxTriList* Data = (dxTriList*)dGeomGetClassData(g);
	Data->Collider->Build();
}

void dGeomTriListGetTriangle(dGeomID g, dword Index, Vertex* v0, Vertex* v1, Vertex* v2){
	dxTriList* Data = (dxTriList*)dGeomGetClassData(g);
	if (v0 != null){
		(*v0) = Data->Collider->Vertices[Data->Collider->Indices[Index * 3 + 0]];
	}
	if (v1 != null){
		(*v1) = Data->Collider->Vertices[Data->Collider->Indices[Index * 3 + 1]];
	}
	if (v2 != null){
		(*v2) = Data->Collider->Vertices[Data->Collider->Indices[Index * 3 + 2]];
	}
}
