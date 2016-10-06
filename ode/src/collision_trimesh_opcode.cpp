/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

// TriMesh code by Erwin de Vries.
// Refactoring into C++ by Oleh Derevenko

#include <ode/collision.h>
#include <ode/rotation.h>
#include "config.h"
#include "matrix.h"
#include "odemath.h"


#if dTRIMESH_ENABLED && dTRIMESH_OPCODE

#include "collision_util.h"
#include "collision_trimesh_opcode.h"


//////////////////////////////////////////////////////////////////////////

struct EdgeRecord
{
public:
    void SetupEdge(int edgeIdx, int triIdx, const dTriIndex* vertIdxs);
    const Point *GetOppositeVert(const Point *vertices[]) const;

public:
    static int CompareEdges(const void* edge1, const void* edge2);

public:
    int m_VertIdx1;	// Index into vertex array for this edges vertices
    int m_VertIdx2;
    int m_TriIdx;		// Index into triangle array for triangle this edge belongs to

    uint8 m_EdgeFlags;	
    uint8 m_Vert1Flags;
    uint8 m_Vert2Flags;
    bool m_Concave;
};


void EdgeRecord::SetupEdge(int edgeIdx, int triIdx, const dTriIndex* vertIdxs)
{
    if (edgeIdx < 1)
    {
        dIASSERT(edgeIdx == 0);

        m_EdgeFlags  = dxTriMeshData::kEdge0;
        m_Vert1Flags = dxTriMeshData::kVert0;
        m_Vert2Flags = dxTriMeshData::kVert1;
        m_VertIdx1 = vertIdxs[0];
        m_VertIdx2 = vertIdxs[1];
    }
    else if (edgeIdx == 1)
    {
        m_EdgeFlags  = dxTriMeshData::kEdge1;
        m_Vert1Flags = dxTriMeshData::kVert1;
        m_Vert2Flags = dxTriMeshData::kVert2;
        m_VertIdx1 = vertIdxs[1];
        m_VertIdx2 = vertIdxs[2];
    }
    else
    {
        dIASSERT(edgeIdx == 2);

        m_EdgeFlags  = dxTriMeshData::kEdge2;
        m_Vert1Flags = dxTriMeshData::kVert2;
        m_Vert2Flags = dxTriMeshData::kVert0;
        m_VertIdx1 = vertIdxs[2];
        m_VertIdx2 = vertIdxs[0];
    }

    // Make sure vert index 1 is less than index 2 (for easier sorting)
    if (m_VertIdx1 > m_VertIdx2)
    {
        unsigned int tempIdx = m_VertIdx1;
        m_VertIdx1 = m_VertIdx2;
        m_VertIdx2 = tempIdx;

        uint8 tempFlags = m_Vert1Flags;
        m_Vert1Flags = m_Vert2Flags;
        m_Vert2Flags = tempFlags;
    }

    m_TriIdx = triIdx;
    m_Concave = false;
}

// Get the vertex opposite this edge in the triangle
const Point *EdgeRecord::GetOppositeVert(const Point *vertices[]) const
{
    const Point *result;

    if ((m_Vert1Flags == dxTriMeshData::kVert0 && m_Vert2Flags == dxTriMeshData::kVert1) ||
        (m_Vert1Flags == dxTriMeshData::kVert1 && m_Vert2Flags == dxTriMeshData::kVert0))
    {
        result = vertices[2];
    }
    else if ((m_Vert1Flags == dxTriMeshData::kVert1 && m_Vert2Flags == dxTriMeshData::kVert2) ||
        (m_Vert1Flags == dxTriMeshData::kVert2 && m_Vert2Flags == dxTriMeshData::kVert1))
    {
        result = vertices[0];
    }
    else
    {
        result = vertices[1];
    }

    return result;
}


// Edge comparison function for qsort
/*static */
int EdgeRecord::CompareEdges(const void* edge1, const void* edge2)
{
    const EdgeRecord *e1 = (const EdgeRecord *)edge1;
    const EdgeRecord *e2 = (const EdgeRecord *)edge2;

    return e1->m_VertIdx1 - e2->m_VertIdx1 == 0
        ? e1->m_VertIdx2 - e2->m_VertIdx2
        : e1->m_VertIdx1 - e2->m_VertIdx1;
}


//////////////////////////////////////////////////////////////////////////
// TrimeshCollidersCache

void TrimeshCollidersCache::initOPCODECaches()
{
    m_RayCollider.SetDestination(&m_Faces);

    /* -- not used
    _PlanesCollider.SetTemporalCoherence(true);
    */

    m_SphereCollider.SetTemporalCoherence(true);
    m_SphereCollider.SetPrimitiveTests(false);

    m_OBBCollider.SetTemporalCoherence(true);

    // no first-contact test (i.e. return full contact info)
    m_AABBTreeCollider.SetFirstContact( false );     
    // temporal coherence only works with "first contact" tests
    m_AABBTreeCollider.SetTemporalCoherence(false);
    // Perform full BV-BV tests (true) or SAT-lite tests (false)
    m_AABBTreeCollider.SetFullBoxBoxTest( true );
    // Perform full Primitive-BV tests (true) or SAT-lite tests (false)
    m_AABBTreeCollider.SetFullPrimBoxTest( true );
    const char* msg;
    if ((msg =m_AABBTreeCollider.ValidateSettings()))
    {
        dDebug (d_ERR_UASSERT, msg, " (%s:%d)", __FILE__,__LINE__);
    }

    /* -- not used
    _LSSCollider.SetTemporalCoherence(false);
    _LSSCollider.SetPrimitiveTests(false);
    _LSSCollider.SetFirstContact(false);
    */
}

void TrimeshCollidersCache::clearOPCODECaches()
{
    m_Faces.Empty();
    m_DefaultSphereCache.TouchedPrimitives.Empty();
    m_DefaultBoxCache.TouchedPrimitives.Empty();
    m_DefaultCapsuleCache.TouchedPrimitives.Empty();
}


//////////////////////////////////////////////////////////////////////////
// Trimesh data

dxTriMeshData::~dxTriMeshData()
{
    if ( m_UseFlags )
    {
        delete [] m_UseFlags;
    }
}

void dxTriMeshData::build(const Point *Vertices, int VertexStide, unsigned VertexCount,
    const IndexedTriangle *Indices, unsigned IndexCount, int TriStride,
    const dReal *in_Normals,
    bool Single)
{
    dAASSERT(IndexCount % 3 == 0);

    m_Mesh.SetNbTriangles(IndexCount / 3);
    m_Mesh.SetNbVertices(VertexCount);
    m_Mesh.SetPointers(Indices, Vertices);
    m_Mesh.SetStrides(TriStride, VertexStide);
    m_Mesh.SetSingle(Single);

    // Build tree
    BuildSettings Settings;
    // recommended in Opcode User Manual
    //Settings.mRules = SPLIT_COMPLETE | SPLIT_SPLATTERPOINTS | SPLIT_GEOMCENTER;
    // used in ODE, why?
    //Settings.mRules = SPLIT_BEST_AXIS;

    // best compromise?
    Settings.mRules = SPLIT_BEST_AXIS | SPLIT_SPLATTER_POINTS | SPLIT_GEOM_CENTER;


    OPCODECREATE TreeBuilder;
    TreeBuilder.mIMesh = &m_Mesh;

    TreeBuilder.mSettings = Settings;
    TreeBuilder.mNoLeaf = true;
    TreeBuilder.mQuantized = false;

    TreeBuilder.mKeepOriginal = false;
    TreeBuilder.mCanRemap = false;



    m_BVTree.Build(TreeBuilder);

    // compute model space AABB
    dVector3 AABBMax, AABBMin;
    AABBMax[0] = AABBMax[1] = AABBMax[2] = (dReal) -dInfinity;
    AABBMin[0] = AABBMin[1] = AABBMin[2] = (dReal) dInfinity;
    if( Single ) {
        const uint8 *verts = (const uint8 *)Vertices;
        for( unsigned i = 0; i < VertexCount; ++i ) {
            const float* v = (const float*)verts;
            if( v[0] > AABBMax[0] ) AABBMax[0] = v[0];
            if( v[1] > AABBMax[1] ) AABBMax[1] = v[1];
            if( v[2] > AABBMax[2] ) AABBMax[2] = v[2];
            if( v[0] < AABBMin[0] ) AABBMin[0] = v[0];
            if( v[1] < AABBMin[1] ) AABBMin[1] = v[1];
            if( v[2] < AABBMin[2] ) AABBMin[2] = v[2];
            verts += VertexStide;
        }
    } else {
        const uint8 *verts = (const uint8 *)Vertices;
        for( unsigned i = 0; i < VertexCount; ++i ) {
            const double* v = (const double*)verts;
            if( v[0] > AABBMax[0] ) AABBMax[0] = (dReal) v[0];
            if( v[1] > AABBMax[1] ) AABBMax[1] = (dReal) v[1];
            if( v[2] > AABBMax[2] ) AABBMax[2] = (dReal) v[2];
            if( v[0] < AABBMin[0] ) AABBMin[0] = (dReal) v[0];
            if( v[1] < AABBMin[1] ) AABBMin[1] = (dReal) v[1];
            if( v[2] < AABBMin[2] ) AABBMin[2] = (dReal) v[2];
            verts += VertexStide;
        }
    }
    m_AABBCenter[0] = (AABBMin[0] + AABBMax[0]) * REAL(0.5);
    m_AABBCenter[1] = (AABBMin[1] + AABBMax[1]) * REAL(0.5);
    m_AABBCenter[2] = (AABBMin[2] + AABBMax[2]) * REAL(0.5);
    m_AABBExtents[0] = AABBMax[0] - m_AABBCenter[0];
    m_AABBExtents[1] = AABBMax[1] - m_AABBCenter[1];
    m_AABBExtents[2] = AABBMax[2] - m_AABBCenter[2];

    // user data (not used by OPCODE)
    m_Normals = in_Normals;

    dIASSERT(m_UseFlags == NULL);
}

void dxTriMeshData::preprocess()
{
    // If this mesh has already been preprocessed, exit
    if (m_UseFlags)
        return;

    unsigned int numTris = m_Mesh.GetNbTriangles();
    size_t numEdges = (size_t)numTris * 3;

    m_UseFlags = new uint8[numTris];
    memset(m_UseFlags, 0, sizeof(uint8) * numTris);

    EdgeRecord *records = new EdgeRecord[numEdges];

    // Make a list of every edge in the mesh
    const IndexedTriangle *tris = m_Mesh.GetTris();
    const unsigned tristride = m_Mesh.GetTriStride();
    unsigned triangleIdx = 0;
    for (size_t edgeIdx = 0; edgeIdx != numEdges; ++triangleIdx, edgeIdx += 3)
    {
        records[edgeIdx + 0].SetupEdge(0, triangleIdx, tris->mVRef);
        records[edgeIdx + 1].SetupEdge(1, triangleIdx, tris->mVRef);
        records[edgeIdx + 2].SetupEdge(2, triangleIdx, tris->mVRef);

        tris = (const IndexedTriangle*)(((uint8 *)tris) + tristride);
    }

    // Sort the edges, so the ones sharing the same verts are beside each other
    qsort(records, numEdges, sizeof(EdgeRecord), &EdgeRecord::CompareEdges);

    // Go through the sorted list of edges and flag all the edges and vertices that we need to use
    for (unsigned int i = 0; i < numEdges; i++)
    {
        EdgeRecord* rec1 = &records[i];
        EdgeRecord* rec2 = 0;
        if (i < numEdges - 1)
            rec2 = &records[i+1];

        if (rec2 &&
            rec1->m_VertIdx1 == rec2->m_VertIdx1 &&
            rec1->m_VertIdx2 == rec2->m_VertIdx2)
        {
            VertexPointers vp;
            ConversionArea vc;
            m_Mesh.GetTriangle(vp, rec1->m_TriIdx, vc);

            // Get the normal of the first triangle
            Point triNorm = (*vp.Vertex[2] - *vp.Vertex[1]) ^ (*vp.Vertex[0] - *vp.Vertex[1]);
            triNorm.Normalize();

            // Get the vert opposite this edge in the first triangle
            const Point *pOppositeVert1 = rec1->GetOppositeVert(vp.Vertex);

            // Get the vert opposite this edge in the second triangle
            m_Mesh.GetTriangle(vp, rec2->m_TriIdx, vc);
            const Point *pOppositeVert2 = rec2->GetOppositeVert(vp.Vertex);

            Point oppositeEdge = *pOppositeVert2 - *pOppositeVert1;
            float dot = triNorm.Dot(oppositeEdge.Normalize());

            // We let the dot threshold for concavity get slightly negative to allow for rounding errors
            const float kConcaveThresh = -0.000001f;

            // This is a concave edge, leave it for the next pass
            if (dot >= kConcaveThresh)
                rec1->m_Concave = true;
            // If this is a convex edge, mark its vertices and edge as used
            else
                m_UseFlags[rec1->m_TriIdx] |= rec1->m_Vert1Flags | rec1->m_Vert2Flags | rec1->m_EdgeFlags;

            // Skip the second edge
            i++;
        }
        // This is a boundary edge
        else
        {
            m_UseFlags[rec1->m_TriIdx] |= rec1->m_Vert1Flags | rec1->m_Vert2Flags | rec1->m_EdgeFlags;
        }
    }

    // Go through the list once more, and take any edge we marked as concave and
    // clear it's vertices flags in any triangles they're used in
    for (unsigned int i = 0; i < numEdges; i++)
    {
        EdgeRecord& er = records[i];

        if (er.m_Concave)
        {
            for (unsigned int j = 0; j < numEdges; j++)
            {
                const EdgeRecord &curER = records[j];

                if (curER.m_VertIdx1 == er.m_VertIdx1 ||
                    curER.m_VertIdx1 == er.m_VertIdx2)
                    m_UseFlags[curER.m_TriIdx] &= ~curER.m_Vert1Flags;

                if (curER.m_VertIdx2 == er.m_VertIdx1 ||
                    curER.m_VertIdx2 == er.m_VertIdx2)
                    m_UseFlags[curER.m_TriIdx] &= ~curER.m_Vert2Flags;
            }
        }
    }

    delete [] records;
}

void dxTriMeshData::updateData()
{
    m_BVTree.Refit();
}


//////////////////////////////////////////////////////////////////////////
// dxTriMesh

dxTriMesh::~dxTriMesh()
{
    //
}

void dxTriMesh::clearTCCache()
{
    /* dxTriMesh::ClearTCCache uses dArray's setSize(0) to clear the caches -
    but the destructor isn't called when doing this, so we would leak.
    So, call the previous caches' containers' destructors by hand first. */
    int i, n;

    n = m_SphereTCCache.size();
    for( i = 0; i != n; ++i ) 
    {
        m_SphereTCCache[i].~SphereTC();
    }
    m_SphereTCCache.setSize(0);

    n = m_BoxTCCache.size();
    for( i = 0; i != n; ++i ) 
    {
        m_BoxTCCache[i].~BoxTC();
    }
    m_BoxTCCache.setSize(0);

    n = m_CapsuleTCCache.size();
    for( i = 0; i != n; ++i ) 
    {
        m_CapsuleTCCache[i].~CapsuleTC();
    }
    m_CapsuleTCCache.setSize(0);
}


bool dxTriMesh::controlGeometry(int controlClass, int controlCode, void *dataValue, int *dataSize)
{
    if (controlClass == dGeomColliderControlClass) 
    {
        if (controlCode == dGeomCommonAnyControlCode) 
        {
            return checkControlValueSizeValidity(dataValue, dataSize, 0);
        }
        else if (controlCode == dGeomColliderSetMergeSphereContactsControlCode) 
        {
            return checkControlValueSizeValidity(dataValue, dataSize, sizeof(int)) 
                && controlGeometry_SetMergeSphereContacts(*(int *)dataValue);
        }
        else if (controlCode == dGeomColliderGetMergeSphereContactsControlCode) 
        {
            return checkControlValueSizeValidity(dataValue, dataSize, sizeof(int)) 
                && controlGeometry_GetMergeSphereContacts(*(int *)dataValue);
        }
    }

    return dxTriMesh_Parent::controlGeometry(controlClass, controlCode, dataValue, dataSize);
}

bool dxTriMesh::controlGeometry_SetMergeSphereContacts(int dataValue)
{
    if (dataValue == dGeomColliderMergeContactsValue__Default) 
    {
        m_SphereContactsMergeOption = (dxContactMergeOptions)MERGE_NORMALS__SPHERE_DEFAULT;
    }
    else if (dataValue == dGeomColliderMergeContactsValue_None) 
    {
        m_SphereContactsMergeOption = DONT_MERGE_CONTACTS;
    }
    else if (dataValue == dGeomColliderMergeContactsValue_Normals) 
    {
        m_SphereContactsMergeOption = MERGE_CONTACT_NORMALS;
    }
    else if (dataValue == dGeomColliderMergeContactsValue_Full) 
    {
        m_SphereContactsMergeOption = MERGE_CONTACTS_FULLY;
    }
    else 
    {
        dAASSERT(false && "Invalid contact merge control value");
        return false;
    }

    return true;
}

bool dxTriMesh::controlGeometry_GetMergeSphereContacts(int &returnValue)
{
    if (m_SphereContactsMergeOption == DONT_MERGE_CONTACTS) {
        returnValue = dGeomColliderMergeContactsValue_None;
    }
    else if (m_SphereContactsMergeOption == MERGE_CONTACT_NORMALS) {
        returnValue = dGeomColliderMergeContactsValue_Normals;
    }
    else if (m_SphereContactsMergeOption == MERGE_CONTACTS_FULLY) {
        returnValue = dGeomColliderMergeContactsValue_Full;
    }
    else {
        dIASSERT(false && "Internal error: unexpected contact merge option field value");
        return false;
    }

    return true;
}


/*virtual */
void dxTriMesh::computeAABB() 
{
    const dxTriMeshData* d = m_Data;
    dVector3 c;
    const dMatrix3& R = final_posr->R;
    const dVector3& pos = final_posr->pos;

    dMultiply0_331( c, R, d->m_AABBCenter );

    dReal xrange = dFabs(R[0] * m_Data->m_AABBExtents[0]) +
        dFabs(R[1] * m_Data->m_AABBExtents[1]) + 
        dFabs(R[2] * m_Data->m_AABBExtents[2]);
    dReal yrange = dFabs(R[4] * m_Data->m_AABBExtents[0]) +
        dFabs(R[5] * m_Data->m_AABBExtents[1]) + 
        dFabs(R[6] * m_Data->m_AABBExtents[2]);
    dReal zrange = dFabs(R[8] * m_Data->m_AABBExtents[0]) +
        dFabs(R[9] * m_Data->m_AABBExtents[1]) + 
        dFabs(R[10] * m_Data->m_AABBExtents[2]);

    aabb[0] = c[0] + pos[0] - xrange;
    aabb[1] = c[0] + pos[0] + xrange;
    aabb[2] = c[1] + pos[1] - yrange;
    aabb[3] = c[1] + pos[1] + yrange;
    aabb[4] = c[2] + pos[2] - zrange;
    aabb[5] = c[2] + pos[2] + zrange;
}


void dxTriMesh::fetchMeshTransformedTriangle(dVector3 *const pout_triangle[3], unsigned index)
{
    const dVector3 &position = buildUpdatedPosition();
    const dMatrix3 &rotation = buildUpdatedRotation();
    fetchMeshTriangle(pout_triangle, index, position, rotation);
}

void dxTriMesh::fetchMeshTransformedTriangle(dVector3 out_triangle[3], unsigned index)
{
    const dVector3 &position = buildUpdatedPosition();
    const dMatrix3 &rotation = buildUpdatedRotation();
    fetchMeshTriangle(out_triangle, index, position, rotation);
}

void dxTriMesh::fetchMeshTriangle(dVector3 *const pout_triangle[3], unsigned index, const dVector3 position, const dMatrix3 rotation) const
{
    dIASSERT(dIN_RANGE(index, 0, getMeshTriangleCount()));

    VertexPointers VP;
    ConversionArea VC;
    m_Data->m_Mesh.GetTriangle(VP, index, VC);

    for (unsigned i = 0; i != 3; ++i)
    {
        if (pout_triangle[i] != NULL)
        {
            dVector3 v;
            v[dV3E_X] = VP.Vertex[i]->x;
            v[dV3E_Y] = VP.Vertex[i]->y;
            v[dV3E_Z] = VP.Vertex[i]->z;

            dVector3 &out_triangle = *(pout_triangle[i]);
            dMultiply0_331(out_triangle, rotation, v);
            dAddVectors3(out_triangle, out_triangle, position);
            out_triangle[dV3E_PAD] = REAL(0.0);
        }
    }
}

void dxTriMesh::fetchMeshTriangle(dVector3 out_triangle[3], unsigned index, const dVector3 position, const dMatrix3 rotation) const
{
    dIASSERT(dIN_RANGE(index, 0, getMeshTriangleCount()));

    VertexPointers VP;
    ConversionArea VC;
    m_Data->m_Mesh.GetTriangle(VP, index, VC);

    for (unsigned i = 0; i != 3; ++i)
    {
        dVector3 v;
        v[dV3E_X] = VP.Vertex[i]->x;
        v[dV3E_Y] = VP.Vertex[i]->y;
        v[dV3E_Z] = VP.Vertex[i]->z;

        dMultiply0_331(out_triangle[i], rotation, v);
        dAddVectors3(out_triangle[i], out_triangle[i], position);
        out_triangle[i][dV3E_PAD] = REAL(0.0);
    }
}


//////////////////////////////////////////////////////////////////////////

/*extern */
dTriMeshDataID dGeomTriMeshDataCreate()
{
    return new dxTriMeshData();
}

/*extern */
void dGeomTriMeshDataDestroy(dTriMeshDataID g)
{
    dxTriMeshData *mesh = g;
    delete mesh;
}


/*extern */
void dGeomTriMeshDataSet(dTriMeshDataID g, int data_id, void* in_data)
{
    dUASSERT(g, "The argument is not a trimesh data");

    dxTriMeshData *data = g;

    switch (data_id)
    {
    case TRIMESH_FACE_NORMALS:
        data->assignNormals((const dReal *)in_data);
        break;

    default:
        dUASSERT(data_id, "invalid data type");
        break;
    }
}

/*extern */
void *dGeomTriMeshDataGet(dTriMeshDataID g, int data_id)
{
    dUASSERT(g, "The argument is not a trimesh data");

    const dxTriMeshData *data = g;

    void *result = NULL;

    switch (data_id)
    {
    case TRIMESH_FACE_NORMALS:
        result = (void *)data->retrieveNormals();
        break;

    default:
        dUASSERT(data_id, "invalid data type");
        break;
    }

    return result;
}


/*extern */
void dGeomTriMeshDataBuildSingle1(dTriMeshDataID g,
    const void* Vertices, int VertexStride, int VertexCount, 
    const void* Indices, int IndexCount, int TriStride,
    const void* Normals)
{
    dUASSERT(g, "The argument is not a trimesh data");

    dxTriMeshData *data = g;
    data->build((const Point *)Vertices, VertexStride, VertexCount, 
        (const IndexedTriangle *)Indices, IndexCount, TriStride, 
        (const dReal *)Normals, 
        true);
}

/*extern */
void dGeomTriMeshDataBuildDouble1(dTriMeshDataID g,
    const void* Vertices, int VertexStride, int VertexCount, 
    const void* Indices, int IndexCount, int TriStride,
    const void* Normals)
{
    dUASSERT(g, "The argument is not a trimesh data");

    g->build((const Point *)Vertices, VertexStride, VertexCount, 
        (const IndexedTriangle *)Indices, IndexCount, TriStride, 
        (const dReal *)Normals, 
        false);
}


/*extern */
void dGeomTriMeshDataGetBuffer(dTriMeshDataID g, unsigned char **buf, int *bufLen)
{
    dUASSERT(g, "The argument is not a trimesh data");

    const dxTriMeshData *data = g;
    *buf = data->retrieveUseFlagsBuffer(*(unsigned int *)bufLen);
}

/*extern */
void dGeomTriMeshDataSetBuffer(dTriMeshDataID g, unsigned char* buf)
{
    dUASSERT(g, "The argument is not a trimesh data");

    dxTriMeshData *data = g;
    data->assignUseFlagsBuffer(buf);
}


//////////////////////////////////////////////////////////////////////////

/*extern */
dGeomID dCreateTriMesh(dSpaceID space, 
    dTriMeshDataID Data,
    dTriCallback* Callback,
    dTriArrayCallback* ArrayCallback,
    dTriRayCallback* RayCallback)
{
    dxTriMesh *mesh = new dxTriMesh(space, Data, Callback, ArrayCallback, RayCallback);
    return mesh;
}


/*extern */
void dGeomTriMeshSetLastTransform(dGeomID g, const dMatrix4 last_trans )
{
    dAASSERT(g);
    dUASSERT(g->type == dTriMeshClass, "The geom is not a trimesh");

    dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
    mesh->assignLastTransform(last_trans);
}

/*extern */
const dReal *dGeomTriMeshGetLastTransform(dGeomID g)
{
    dAASSERT(g);
    dUASSERT(g->type == dTriMeshClass, "The geom is not a trimesh");

    dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
    return mesh->retrieveLastTransform();
}


//////////////////////////////////////////////////////////////////////////

// Cleanup for allocations when shutting down ODE
/*extern */
void opcode_collider_cleanup()
{
#if !dTLS_ENABLED

    // Clear TC caches
    TrimeshCollidersCache *pccColliderCache = GetTrimeshCollidersCache(0);
    pccColliderCache->clearOPCODECaches();

#endif // dTLS_ENABLED
}


#endif // dTRIMESH_ENABLED && dTRIMESH_OPCODE

