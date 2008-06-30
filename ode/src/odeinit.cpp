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

/*

ODE initialization/finalization code

*/

#include <ode/common.h>
#include <ode/odemath.h>
#include <ode/odeinit.h>
#include "config.h"
#include "collision_kernel.h"
#include "collision_trimesh_internal.h"
#include "odetls.h"
#include "odeou.h"


//****************************************************************************
// Thread local data allocators and providers

#if dTLS_ENABLED
#if dTRIMESH_ENABLED 

#if dTRIMESH_OPCODE

static Opcode::ThreadLocalData *ProvideOpcodeThreadLocalData()
{
	Opcode::ThreadLocalData *pldOpcodeData = (Opcode::ThreadLocalData *)COdeTls::GetTrimeshCollisionLibraryData();
	return pldOpcodeData;
}

#endif // dTRIMESH_OPCODE

#if dTRIMESH_GIMPACT

// No thread local data provider for GIMPACT

#endif // dTRIMESH_GIMPACT

#endif // dTRIMESH_ENABLED
#endif // dTLS_ENABLED


enum
{
	TLD_INTERNAL_COLLISIONDATA_ALLOCATED = 0x00000001,
};

static bool AllocateThreadBasicDataIfNecessary()
{
	bool bResult = false;
	
	do
	{
#if dTLS_ENABLED

		const unsigned uDataAllocationFlags = COdeTls::GetDataAllocationFlags();

		// If no flags are set it may mean that TLS slot is not allocated yet
		if (uDataAllocationFlags == 0)
		{
			// Assign zero flags to make sure that TLS slot has been allocated
			if (!COdeTls::AssignDataAllocationFlags(0))
			{
				break;
			}
		}

#endif // #if dTLS_ENABLED

		bResult = true;
	}
	while (false);
	
	return bResult;
}

static void FreeThreadBasicDataOnFailureIfNecessary()
{
#if dTLS_ENABLED

	const unsigned uDataAllocationFlags = COdeTls::GetDataAllocationFlags();

	if (uDataAllocationFlags == 0)
	{
		// So far, only free TLS slot, if no subsystems have data allocated
		COdeTls::CleanupForThread();
	}

#endif // #if dTLS_ENABLED
}

#if dTLS_ENABLED
static bool AllocateThreadCollisionData()
{
	bool bResult = false;

	bool bCollidersCacheAllocated = false, bCollisionLibraryDataAllocated = false;

	do
	{
		dIASSERT(!(COdeTls::GetDataAllocationFlags() & TLD_INTERNAL_COLLISIONDATA_ALLOCATED));

#if dTRIMESH_ENABLED 

		TrimeshCollidersCache *pccColliderCache = new TrimeshCollidersCache();
		if (!COdeTls::AssignTrimeshCollidersCache(pccColliderCache))
		{
			delete pccColliderCache;
			break;
		}

		bCollidersCacheAllocated = true;

#if dTRIMESH_OPCODE

		Opcode::ThreadLocalData *pldOpcodeData = new Opcode::ThreadLocalData();
		if (!COdeTls::AssignTrimeshCollisionLibraryData((void *)pldOpcodeData))
		{
			delete pldOpcodeData;
			break;
		}

#endif // dTRIMESH_OPCODE

#if dTRIMESH_GIMPACT

		// N thread local data for GIMPACT

#endif // dTRIMESH_GIMPACT

		bCollisionLibraryDataAllocated = true;

#endif // dTRIMESH_ENABLED

		COdeTls::SignalDataAllocationFlags(TLD_INTERNAL_COLLISIONDATA_ALLOCATED);

		bResult = true;
	}
	while (false);

	if (!bResult)
	{
		if (bCollisionLibraryDataAllocated)
		{
			COdeTls::DestroyTrimeshCollisionLibraryData();
		}

		if (bCollidersCacheAllocated)
		{
			COdeTls::DestroyTrimeshCollidersCache();
		}
	}
	
	return bResult;
}
#endif // dTLS_ENABLED

static bool AllocateThreadCollisionDataIfNecessary(bool &bOutDataAllocated)
{
	bool bResult = false;
	bOutDataAllocated = false;

	do 
	{
#if dTLS_ENABLED

		const unsigned uDataAllocationFlags = COdeTls::GetDataAllocationFlags();

		if ((uDataAllocationFlags & TLD_INTERNAL_COLLISIONDATA_ALLOCATED) == 0)
		{
			if (!AllocateThreadCollisionData())
			{
				break;
			}

			bOutDataAllocated = true;
		}

#endif // #if dTLS_ENABLED

		bResult = true;
	}
	while (false);

	return bResult;
}

static void FreeThreadCollisionData()
{
#if dTLS_ENABLED

	COdeTls::DestroyTrimeshCollisionLibraryData();
	COdeTls::DestroyTrimeshCollidersCache();

	COdeTls::DropDataAllocationFlags(TLD_INTERNAL_COLLISIONDATA_ALLOCATED);

#endif // dTLS_ENABLED
}


//****************************************************************************
// initialization and shutdown routines - allocate and initialize data,
// cleanup before exiting

static bool g_bODEInitialized = false;

void dInitODE()
{
	int bInitResult = dInitODE2(0);
	dIASSERT(bInitResult); dVARIABLEUSED(bInitResult);

	int ibAllocResult = dAllocateODEDataForThread(dAllocateMaskAll);
	dIASSERT(ibAllocResult); dVARIABLEUSED(ibAllocResult);
}

int dInitODE2(unsigned int uiInitFlags/*=0*/)
{
	dIASSERT(!g_bODEInitialized); // ODE can not be initialized twice

	bool bResult = false;
	
#if dOU_ENABLED
	bool bOUCustomizationsDone = false;
#endif
#if dATOMICS_ENABLED
	bool bAtomicsInitialized = false;
#endif
#if dTLS_ENABLED
	bool bTlsInitialized = false;
#endif

	do
	{
#if dOU_ENABLED
		if (!COdeOu::DoOUCustomizations())
		{
			break;
		}

		bOUCustomizationsDone = true;
#endif

#if dATOMICS_ENABLED
		if (!COdeOu::InitializeAtomics())
		{
			break;
		}

		bAtomicsInitialized = true;
#endif

#if dTLS_ENABLED
		unsigned int uiTlsFlags = 0;

		if (uiInitFlags & dInitFlagManualThreadCleanup)
		{
			uiTlsFlags |= COdeTls::MANUAL_DATA_CLEANUP;
		}

		if (!COdeTls::Initialize(uiTlsFlags))
		{
			break;
		}

		bTlsInitialized = true;
#endif
		
#if dTRIMESH_ENABLED && dTRIMESH_OPCODE
		Opcode::ThreadLocalDataProviderProc pfnOpcodeDataProviderProc;
#if dTLS_ENABLED
		pfnOpcodeDataProviderProc = &ProvideOpcodeThreadLocalData;
#else // dTLS_ENABLED
		pfnOpcodeDataProviderProc = NULL;
#endif // dTLS_ENABLED
		if (!Opcode::InitOpcode(pfnOpcodeDataProviderProc))
		{
			break;
		}
#endif

#if dTRIMESH_ENABLED && dTRIMESH_GIMPACT
		gimpact_init();
#endif

		dInitColliders();

		g_bODEInitialized = true;
		bResult = true;
	}
	while (false);

	if (!bResult)
	{
#if dTLS_ENABLED
		if (bTlsInitialized)
		{
			COdeTls::Finalize();
		}
#endif

#if dATOMICS_ENABLED
		if (bAtomicsInitialized)
		{
			COdeOu::FinalizeAtomics();
		}
#endif

#if dOU_ENABLED
		if (bOUCustomizationsDone)
		{
			COdeOu::UndoOUCustomizations();
		}
#endif
	}

	return bResult;
}


int dAllocateODEDataForThread(unsigned int uiAllocateFlags)
{
	dIASSERT(g_bODEInitialized); // Call dInitODEEx first

	bool bResult = false;
	
	bool bCollisionDataAllocated = false;

	do
	{
		if (!AllocateThreadBasicDataIfNecessary())
		{
			break;
		}

		if (uiAllocateFlags & dAllocateFlagCollisionData)
		{
			if (!AllocateThreadCollisionDataIfNecessary(bCollisionDataAllocated))
			{
				break;
			}
		}
	
		bResult = true;
	}
	while (false);

	if (!bResult)
	{
		if (bCollisionDataAllocated)
		{
			FreeThreadCollisionData();
		}

		FreeThreadBasicDataOnFailureIfNecessary();
	}
	
	return bResult;
}

void dCleanupODEAllDataForThread()
{
	dIASSERT(g_bODEInitialized); // Call dInitODEEx first or delay dCloseODE until all threads exit

#if dTLS_ENABLED
	COdeTls::CleanupForThread();
#endif
}


void dCloseODE()
{
	dIASSERT(g_bODEInitialized); // dCloseODE must not be called without dInitODEEx or if dInitODEEx fails

	g_bODEInitialized = false;

	dClearPosrCache();
	dFinitUserClasses();
	dFinitColliders();

#if dTRIMESH_ENABLED && dTRIMESH_GIMPACT
	gimpact_terminate();
#endif

#if dTRIMESH_ENABLED && dTRIMESH_OPCODE
	extern void opcode_collider_cleanup();
	// Free up static allocations in opcode
	opcode_collider_cleanup();

	Opcode::CloseOpcode();
#endif

#if dTLS_ENABLED
	COdeTls::Finalize();
#endif

#if dATOMICS_ENABLED
	COdeOu::FinalizeAtomics();
#endif

#if dOU_ENABLED
	COdeOu::UndoOUCustomizations();
#endif
}

