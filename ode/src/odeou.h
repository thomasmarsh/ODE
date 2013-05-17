/*************************************************************************
*                                                                       *
* OU library interface file for Open Dynamics Engine,                   *
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

ODE interface to OU library functions.

*/


#ifndef _ODE_ODEOU_H_
#define _ODE_ODEOU_H_


#if dOU_ENABLED

#include <ou/assert.h>
#include <ou/enumarrays.h>
#include <ou/macros.h>
#include <ou/templates.h>
#include <ou/typewrapper.h>
#include <ou/simpleflags.h>
#include <ou/customization.h>

#if dATOMICS_ENABLED
#include <ou/atomic.h>
#include <ou/atomicflags.h>
#endif

#if dTLS_ENABLED
#include <ou/threadlocalstorage.h>
#endif


using _OU_NAMESPACE::CEnumUnsortedElementArray;

#if dATOMICS_ENABLED
using _OU_NAMESPACE::atomicord32;
using _OU_NAMESPACE::atomicptr;
using _OU_NAMESPACE::InitializeAtomicAPI;
using _OU_NAMESPACE::FinalizeAtomicAPI;
using _OU_NAMESPACE::AtomicCompareExchange;
using _OU_NAMESPACE::AtomicExchange;
using _OU_NAMESPACE::AtomicCompareExchangePointer;
using _OU_NAMESPACE::AtomicExchangePointer;
#endif


class COdeOu
{
public:
    static bool DoOUCustomizations();
    static void UndoOUCustomizations();

#if dATOMICS_ENABLED
    static bool InitializeAtomics() { return InitializeAtomicAPI(); }
    static void FinalizeAtomics() { FinalizeAtomicAPI(); }
#endif
};


#else // !dOU_ENABLED

typedef unsigned int atomicord32;
typedef size_t atomicptr;


#endif // dOU_ENABLED


#if !dATOMICS_ENABLED

static inline 
bool AtomicCompareExchange(volatile atomicord32 *paoDestination, atomicord32 aoComparand, atomicord32 aoExchange)
{
    return (*paoDestination == aoComparand) ? ((*paoDestination = aoExchange), true) : false;
}

static inline 
atomicord32 AtomicExchange(volatile atomicord32 *paoDestination, atomicord32 aoExchange)
{
    atomicord32 aoDestinationValue = *paoDestination;
    *paoDestination = aoExchange;
    return aoDestinationValue;
}

static inline 
bool AtomicCompareExchangePointer(volatile atomicptr *papDestination, atomicptr apComparand, atomicptr apExchange)
{
    return (*papDestination == apComparand) ? ((*papDestination = apExchange), true) : false;
}

static inline 
atomicptr AtomicExchangePointer(volatile atomicptr *papDestination, atomicptr apExchange)
{
    atomicptr apDestinationValue = *papDestination;
    *papDestination = apExchange;
    return apDestinationValue;
}

#endif // #if !dATOMICS_ENABLED


static inline 
unsigned int AtomicIncrementIntUpToLimit(volatile unsigned int *storagePointer, unsigned int limitValue)
{
    unsigned int resultValue;
    while (true) {
        resultValue = *storagePointer;
        if (resultValue == limitValue) {
            break;
        }
        if (AtomicCompareExchange((volatile atomicord32 *)storagePointer, (atomicord32)resultValue, (atomicord32)(resultValue + 1))) {
            break;
        }
    }
    return resultValue;
}

static inline 
size_t AtomicIncrementSizeUpToLimit(volatile size_t *storagePointer, size_t limitValue)
{
    size_t resultValue;
    while (true) {
        resultValue = *storagePointer;
        if (resultValue == limitValue) {
            break;
        }
        if (AtomicCompareExchangePointer((volatile atomicptr *)storagePointer, (atomicptr)resultValue, (atomicptr)(resultValue + 1))) {
            break;
        }
    }
    return resultValue;
}


#endif // _ODE_ODEOU_H_
