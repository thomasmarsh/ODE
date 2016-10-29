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
// Modified for FreeSOLID Compatibility by Rodrigo Hernandez
// Trimesh caches separation by Oleh Derevenko


#ifndef _ODE_COLLISION_TRIMESH_INTERNAL_H_
#define _ODE_COLLISION_TRIMESH_INTERNAL_H_


//****************************************************************************
// dxTriMesh class


#include "collision_kernel.h"
#include "collision_trimesh_colliders.h"
#include "collision_util.h"
#include <ode/collision_trimesh.h>

#if dTLS_ENABLED
#include "odetls.h"
#endif

#include "util.h"


struct TrimeshCollidersCache;
struct dxTriMeshData;


static inline 
TrimeshCollidersCache *GetTrimeshCollidersCache(unsigned uiTLSKind)
{
#if dTLS_ENABLED
    EODETLSKIND tkTLSKind = (EODETLSKIND)uiTLSKind;
    return COdeTls::GetTrimeshCollidersCache(tkTLSKind);
#else // dTLS_ENABLED
    (void)uiTLSKind; // unused
    extern TrimeshCollidersCache g_ccTrimeshCollidersCache;
    return &g_ccTrimeshCollidersCache;
#endif // dTLS_ENABLED
}


typedef dBase dxTriDataBase_Parent;
struct dxTriDataBase:
    public dxTriDataBase_Parent
{
public:
    dxTriDataBase():
        dxTriDataBase_Parent(),
        m_Vertices(NULL),
        m_VertexStride(0),
        m_VertexCount(0),
        m_Indices(NULL),
        m_TriangleCount(0),
        m_TriStride(0),
        m_Single(false)
    {
#if !dTRIMESH_ENABLED
        dUASSERT(false, "dTRIMESH_ENABLED is not defined. Trimesh geoms will not work");
#endif
    }

    ~dxTriDataBase();

    void buildData(const void *Vertices, int VertexStide, unsigned VertexCount, 
        const void *Indices, unsigned IndexCount, int TriStride, 
        const void *Normals, 
        bool Single);


    /* Array of flags for which edges and vertices should be used on each triangle */
    enum m_UseFlags
    {
        kEdge0 = 0x1,
        kEdge1 = 0x2,
        kEdge2 = 0x4,
        kVert0 = 0x8,
        kVert1 = 0x10,
        kVert2 = 0x20,

        kUseAll = 0xFF
    };

public:
    void assignNormals(const void *normals) { m_Normals = normals; }
    const void *retrieveNormals() const { return m_Normals; }

public:
    const void *m_Vertices;
    int m_VertexStride;
    unsigned m_VertexCount;
    const void *m_Indices;
    unsigned m_TriangleCount;
    int m_TriStride;
    bool m_Single;

private:
    const void *m_Normals;
};


typedef dxGeom dxMeshBase_Parent;
struct dxMeshBase:
    public dxMeshBase_Parent
{
public:
    dxMeshBase(dxSpace *Space, dxTriMeshData *Data, 
        dTriCallback *Callback, dTriArrayCallback *ArrayCallback, dTriRayCallback *RayCallback, 
        bool doTCs=false):
        dxMeshBase_Parent(Space, 1),
        m_Callback(Callback),
        m_ArrayCallback(ArrayCallback),
        m_RayCallback(RayCallback),
        m_TriMergeCallback(NULL),
        m_Data(Data)
    {
        std::fill(m_DoTCs, m_DoTCs + dARRAY_SIZE(m_DoTCs), doTCs);
        type = dTriMeshClass;
    }

    bool invokeCallback(dxGeom *Object, int TriIndex)
    {
        return m_Callback == NULL || m_Callback(this, Object, TriIndex) != 0;
    }

public:
    enum TRIMESHTC
    {
        TTC__MIN,

        TTC_SPHERE = TTC__MIN,
        TTC_BOX,
        TTC_CAPSULE,

        TTC__MAX,
    };

public:
    void assignCallback(dTriCallback *value) { m_Callback = value; }
    dTriCallback *retrieveCallback() const { return m_Callback; }

    void assignArrayCallback(dTriArrayCallback *value) { m_ArrayCallback = value; }
    dTriArrayCallback *retrieveArrayCallback() const { return m_ArrayCallback; }

    void assignRayCallback(dTriRayCallback *value) { m_RayCallback = value; }
    dTriRayCallback *retrieveRayCallback() const { return m_RayCallback; }

    void assignTriMergeCallback(dTriTriMergeCallback *value) { m_TriMergeCallback = value; }
    dTriTriMergeCallback *retrieveTriMergeCallback() const { return m_TriMergeCallback; }

    void assignMeshData(dxTriMeshData *instance)
    {
        setMeshData(instance);
        // I changed my data -- I know nothing about my own AABB anymore.
        markAABBBad();
    }
    dxTriMeshData *retrieveMeshData() const { return getMeshData(); }

    void assignDoTC(TRIMESHTC tc, bool value) { setDoTC(tc, value); }
    bool retrieveDoTC(TRIMESHTC tc) const { return getDoTC(tc); }

public:
    void setDoTC(TRIMESHTC tc, bool value) { dIASSERT(dIN_RANGE(tc, TTC__MIN, TTC__MAX)); m_DoTCs[tc] = value; }
    bool getDoTC(TRIMESHTC tc) const { dIASSERT(dIN_RANGE(tc, TTC__MIN, TTC__MAX)); return m_DoTCs[tc]; }

private:
    void setMeshData(dxTriMeshData *Data) { m_Data = Data; }
    dxTriMeshData *getMeshData() const { return m_Data; }

public:
    // Callbacks
    dTriCallback *m_Callback;
    dTriArrayCallback *m_ArrayCallback;
    dTriRayCallback *m_RayCallback;
    dTriTriMergeCallback *m_TriMergeCallback;

    // Data types
    dxTriMeshData *m_Data;

    bool m_DoTCs[TTC__MAX];
};


#include "collision_trimesh_gimpact.h"
#include "collision_trimesh_opcode.h"


#endif	//_ODE_COLLISION_TRIMESH_INTERNAL_H_
