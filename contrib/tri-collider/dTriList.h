/* The only external header */

/* Class ID */
extern int dTriListClass;

/* Per triangle callback */
typedef int dTriCallback(dGeomID TriList, dGeomID RefObject, dword TriangleIndex);
void dGeomTriListSetCallback(dGeomID g, dTriCallback* Callback);
dTriCallback* dGeomTriListGetCallback(dGeomID g);

/* Per object callback */
typedef void dTriArrayCallback(dGeomID TriList, dGeomID RefObject, dcVector<dword>& TriIndices);
void dGeomTriListSetArrayCallback(dGeomID g, dTriArrayCallback* ArrayCallback);
dTriArrayCallback* dGeomTriListGetArrayCallback(dGeomID g);

/* Construction */
dxGeom* dCreateTriList(dSpaceID space, dTriCallback* Callback, dTriArrayCallback* ArrayCallback);

/* Setting data */
dcArray<Vertex>& dGeomTriListGetVertexArray(dGeomID g);
dcArray<dword>& dGeomTriListGetIndexArray(dGeomID g);
void dGeomTriListBuild(dGeomID g);

/* Getting data */
void dGeomTriListGetTriangle(dGeomID g, dword Index, Vertex* v0, Vertex* v1, Vertex* v2);
