///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 *	OPCODE - Optimized Collision Detection
 *	Copyright (C) 2001 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/Opcode.htm
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Main file for Opcode.dll.
 *	\file		Opcode.cpp
 *	\author		Pierre Terdiman
 *	\date		March, 20, 2001
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
	Finding a good name is difficult!
	Here's the draft for this lib.... Spooky, uh?

	VOID?			Very Optimized Interference Detection
	ZOID?			Zappy's Optimized Interference Detection
	CID?			Custom/Clever Interference Detection
	AID / ACID!		Accurate Interference Detection
	QUID?			Quick Interference Detection
	RIDE?			Realtime Interference DEtection
	WIDE?			Wicked Interference DEtection (....)
	GUID!
	KID !			k-dop interference detection :)
	OPCODE!			OPtimized COllision DEtection
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Precompiled Header
#include "Stdafx.h"

using namespace Opcode;

struct ThreadLocalDataAsSingleton:
	public ThreadLocalData
{
public:
	ThreadLocalDataAsSingleton(): ThreadLocalData(false) {}

	void Init() { ThreadLocalData::Init(); }
	void Finit() { ThreadLocalData::Finit(); }
};


static ThreadLocalDataAsSingleton g_ThreadLocalDataSingleton;
/*extern */ThreadLocalDataProviderProc g_pfnThreadLocalDataProvider = NULL;

static ThreadLocalData *ProvideThreadLocalDataAsSingleton()
{
	return &g_ThreadLocalDataSingleton;
}



bool Opcode::InitOpcode(ThreadLocalDataProviderProc pfnThreadLocalDataProvider)
{
	//Log("// Initializing OPCODE\n\n");
//	LogAPIInfo();
	
	if (pfnThreadLocalDataProvider)
	{
		g_pfnThreadLocalDataProvider = pfnThreadLocalDataProvider;
	}
	else
	{
		g_ThreadLocalDataSingleton.Init();
		g_pfnThreadLocalDataProvider = &ProvideThreadLocalDataAsSingleton;
	}

	return true;
}

bool Opcode::CloseOpcode()
{
	//Log("// Closing OPCODE\n\n");

	g_pfnThreadLocalDataProvider = NULL;
	g_ThreadLocalDataSingleton.Finit(); // Finit can be safely called without init

	return true;
}


#ifdef ICE_MAIN

void ModuleAttach(HINSTANCE hinstance)
{
}

void ModuleDetach()
{
}

#endif
