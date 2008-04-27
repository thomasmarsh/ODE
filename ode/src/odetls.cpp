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

ODE Thread Local Storage access stub implementation.

*/

#include <ode/common.h>
#include <ode/odemath.h>
#include "config.h"
#include "odetls.h"
#include "collision_trimesh_internal.h"


#if dTLS_ENABLED

//////////////////////////////////////////////////////////////////////////
// Class static fields

HTLSKEY COdeTls::m_htkStorageKey = 0;


//////////////////////////////////////////////////////////////////////////
// Initialization and finalization

bool COdeTls::Initialize(unsigned uFlags/*=0*/)
{
	unsigned uOUFlags = 0;

	if (uFlags & MANUAL_DATA_CLEANUP)
	{
		uOUFlags |= CTLSInitialization::SIF_MANUAL_CLEANUP_ON_THREAD_EXIT;
	}

	bool bResult = CTLSInitialization::InitializeTLSAPI(m_htkStorageKey, OTI__MAX, uOUFlags);
	return bResult;
}

void COdeTls::Finalize()
{
	CTLSInitialization::FinalizeTLSAPI();

	m_htkStorageKey = 0;
}


void COdeTls::CleanupForThread()
{
	CTLSInitialization::CleanupOnThreadExit();
}


//////////////////////////////////////////////////////////////////////////
// Value modifiers

bool COdeTls::AssignDataAllocationFlags(unsigned uInitializationFlags)
{
	bool bResult = CThreadLocalStorage::SetStorageValue(m_htkStorageKey, OTI_DATA_ALLOCATION_FLAGS, (tlsvaluetype)uInitializationFlags);
	return bResult;
}


bool COdeTls::AssignTrimeshCollidersCache(TrimeshCollidersCache *pccInstance)
{
	dIASSERT(!CThreadLocalStorage::GetStorageValue(m_htkStorageKey, OTI_TRIMESH_TRIMESH_COLLIDER_CACHE));

	bool bResult = CThreadLocalStorage::SetStorageValue(m_htkStorageKey, OTI_TRIMESH_TRIMESH_COLLIDER_CACHE, (tlsvaluetype)pccInstance, &COdeTls::FreeTrimeshCollidersCache_Callback);
	return bResult;
}

void COdeTls::DestroyTrimeshCollidersCache()
{
	TrimeshCollidersCache *pccCacheInstance = (TrimeshCollidersCache *)CThreadLocalStorage::GetStorageValue(m_htkStorageKey, OTI_TRIMESH_TRIMESH_COLLIDER_CACHE);

	if (pccCacheInstance)
	{
		FreeTrimeshCollidersCache(pccCacheInstance);

		CThreadLocalStorage::UnsafeSetStorageValue(m_htkStorageKey, OTI_TRIMESH_TRIMESH_COLLIDER_CACHE, (tlsvaluetype)NULL);
	}
}


bool COdeTls::AssignTrimeshCollisionLibraryData(void *pv_DataInstance)
{
#if dTRIMESH_ENABLED
	dIASSERT(!CThreadLocalStorage::GetStorageValue(m_htkStorageKey, OTI_TRIMESH_COLLISION_LIBRARY_DATA));

	bool bResult = CThreadLocalStorage::SetStorageValue(m_htkStorageKey, OTI_TRIMESH_COLLISION_LIBRARY_DATA, (tlsvaluetype)pv_DataInstance, &COdeTls::FreeTrimeshCollisionLibraryData_Callback);
	return bResult;
#else
	dIASSERT(false); // Should only be called when trimesh is enabled
	return false;
#endif
}

void COdeTls::DestroyTrimeshCollisionLibraryData()
{
#if dTRIMESH_ENABLED
	void *pv_DataInstance = (void *)CThreadLocalStorage::GetStorageValue(m_htkStorageKey, OTI_TRIMESH_COLLISION_LIBRARY_DATA);

	if (pv_DataInstance)
	{
		FreeTrimeshCollisionLibraryData(pv_DataInstance);

		CThreadLocalStorage::UnsafeSetStorageValue(m_htkStorageKey, OTI_TRIMESH_COLLISION_LIBRARY_DATA, (tlsvaluetype)NULL);
	}
#else
	dIASSERT(false); // Should only be called when trimesh is enabled
#endif
}


//////////////////////////////////////////////////////////////////////////
// Value type destructors

void COdeTls::FreeTrimeshCollidersCache(TrimeshCollidersCache *pccCacheInstance)
{
	delete pccCacheInstance;
}

void COdeTls::FreeTrimeshCollisionLibraryData(void *pv_DataInstance)
{
#if dTRIMESH_ENABLED

#if dTRIMESH_OPCODE
	Opcode::ThreadLocalData *pldOpcodeData = (Opcode::ThreadLocalData *)pv_DataInstance;
	delete pldOpcodeData;
#endif // dTRIMESH_OPCODE

#if dTRIMESH_GIMPACT
	dIASSERT(false); // There is no library specific data for GIMPACT
#endif // dTRIMESH_GIMPACT

#else
	dIASSERT(false); // Should only be called when trimesh is enabled
#endif
}


//////////////////////////////////////////////////////////////////////////
// Value type destructor callbacks

void COdeTls::FreeTrimeshCollidersCache_Callback(tlsvaluetype vValueData)
{
	TrimeshCollidersCache *pccCacheInstance = (TrimeshCollidersCache *)vValueData;
	FreeTrimeshCollidersCache(pccCacheInstance);
}

void COdeTls::FreeTrimeshCollisionLibraryData_Callback(tlsvaluetype vValueData)
{
#if dTRIMESH_ENABLED
	void *pv_DataInstance = (void *)vValueData;
	FreeTrimeshCollisionLibraryData(pv_DataInstance);
#else
	dIASSERT(false); // Should only be called when trimesh is enabled
#endif
}


#endif // #if dTLS_ENABLED

