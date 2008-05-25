/*************************************************************************
*                                                                       *
* Thread local storage access stub for Open Dynamics Engine,            *
* Copyright (C) 2008 Oleh Derevenko. All rights reserved.               *
* Email: odar@eleks.com (change all "a" to "e")                         *
*                                                                       *
* Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
* All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
*                                                                       *
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

/*

ODE Thread Local Storage access stub interface.

*/


#ifndef _ODE_ODETLS_H_
#define _ODE_ODETLS_H_


#include "odeou.h"


#if dTLS_ENABLED


struct TrimeshCollidersCache;

enum EODETLSITEM
{
	OTI_DATA_ALLOCATION_FLAGS,
	OTI_TRIMESH_TRIMESH_COLLIDER_CACHE,
	OTI_TRIMESH_COLLISION_LIBRARY_DATA,

	OTI__MAX,
};


class COdeTls
{
public:
	enum // Initialize() flags
	{
		MANUAL_DATA_CLEANUP = 0x00000001,
	};

	static bool Initialize(unsigned uFlags=0);
	static void Finalize();

	static void CleanupForThread();

public:
	static unsigned GetDataAllocationFlags()
	{
		// Must be a safe call as it is used to test if TLS slot is allocated at all
		return (unsigned)(size_t)CThreadLocalStorage::GetStorageValue(m_htkStorageKey, OTI_DATA_ALLOCATION_FLAGS);
	}

	static void SignalDataAllocationFlags(unsigned uFlagsMask)
	{
		unsigned uCurrentFlags = (unsigned)(size_t)CThreadLocalStorage::UnsafeGetStorageValue(m_htkStorageKey, OTI_DATA_ALLOCATION_FLAGS);
		CThreadLocalStorage::UnsafeSetStorageValue(m_htkStorageKey, OTI_DATA_ALLOCATION_FLAGS, (tlsvaluetype)(size_t)(uCurrentFlags | uFlagsMask));
	}

	static void DropDataAllocationFlags(unsigned uFlagsMask)
	{
		unsigned uCurrentFlags = (unsigned)(size_t)CThreadLocalStorage::UnsafeGetStorageValue(m_htkStorageKey, OTI_DATA_ALLOCATION_FLAGS);
		CThreadLocalStorage::UnsafeSetStorageValue(m_htkStorageKey, OTI_DATA_ALLOCATION_FLAGS, (tlsvaluetype)(size_t)(uCurrentFlags & ~uFlagsMask));
	}

	static TrimeshCollidersCache *GetTrimeshCollidersCache()
	{ 
		return (TrimeshCollidersCache *)CThreadLocalStorage::UnsafeGetStorageValue(m_htkStorageKey, OTI_TRIMESH_TRIMESH_COLLIDER_CACHE);
	}

	static void *GetTrimeshCollisionLibraryData()
	{
		return (void *)CThreadLocalStorage::UnsafeGetStorageValue(m_htkStorageKey, OTI_TRIMESH_COLLISION_LIBRARY_DATA);
	}

public:
	static bool AssignDataAllocationFlags(unsigned uInitializationFlags);

	static bool AssignTrimeshCollidersCache(TrimeshCollidersCache *pccInstance);
	static void DestroyTrimeshCollidersCache();

	static bool AssignTrimeshCollisionLibraryData(void *pv_DataInstance);
	static void DestroyTrimeshCollisionLibraryData();

private:
	static void FreeTrimeshCollidersCache(TrimeshCollidersCache *pccCacheInstance);
	static void FreeTrimeshCollisionLibraryData(void *pv_DataInstance);

private:
	static void _OU_CONVENTION_CALLBACK FreeTrimeshCollidersCache_Callback(tlsvaluetype vValueData);
	static void _OU_CONVENTION_CALLBACK FreeTrimeshCollisionLibraryData_Callback(tlsvaluetype vValueData);

private:
	static HTLSKEY				m_htkStorageKey;
};


#endif // dTLS_ENABLED


#endif // _ODE_ODETLS_H_
