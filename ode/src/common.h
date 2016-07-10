/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
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

#ifndef _ODE_PRIVATE_COMMON_H_
#define _ODE_PRIVATE_COMMON_H_


#include "typedefs.h"
#include <algorithm>


#define dMACRO_MAX(a, b) ((a) > (b) ? (a) : (b))
#define dMACRO_MIN(a, b) ((a) < (b) ? (a) : (b))



template<size_t tsiTypeSize>
struct _sized_signed;

template<>
struct _sized_signed<sizeof(uint8)>
{
    typedef int8 type;
};

template<>
struct _sized_signed<sizeof(uint16)>
{
    typedef int16 type;
};

template<>
struct _sized_signed<sizeof(uint32)>
{
    typedef int32 type;
};

template<>
struct _sized_signed<sizeof(uint64)>
{
    typedef int64 type;
};

template<typename tintergraltype>
struct _make_signed
{
    typedef typename _sized_signed<sizeof(tintergraltype)>::type type;
};


template<size_t tsiTypeSize>
struct _sized_unsigned;

template<>
struct _sized_unsigned<sizeof(int8)>
{
    typedef uint8 type;
};

template<>
struct _sized_unsigned<sizeof(int16)>
{
    typedef uint16 type;
};

template<>
struct _sized_unsigned<sizeof(int32)>
{
    typedef uint32 type;
};

template<>
struct _sized_unsigned<sizeof(int64)>
{
    typedef uint64 type;
};

template<typename tintergraltype>
struct _make_unsigned
{
    typedef typename _sized_unsigned<sizeof(tintergraltype)>::type type;
};


// template<typename tvalueint, typename tminint, typename tmaxint>
// inline 
// bool dxInRange(tvalueint viValue, tminint miMin, tmaxint miMax)
// {
//     return (typename _sized_unsigned<dMACRO_MAX(sizeof(tvalueint), sizeof(tminint))>::type)(viValue - miMin) < (typename _sized_unsigned<dMACRO_MAX(sizeof(tmaxint), sizeof(tminint))>::type)(miMax - miMin);
// }
// #define dIN_RANGE(aval, amin, amax) dxInRange(aval, amin, amax)

#if defined(__GNUC__)
#define __dIN_RANGE_TYPENAME__ typename
#else
#define __dIN_RANGE_TYPENAME__
#endif
#define dIN_RANGE(aval, amin, amax) ((__dIN_RANGE_TYPENAME__ _sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)((__dIN_RANGE_TYPENAME__ _sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)(aval) - (__dIN_RANGE_TYPENAME__ _sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)(amin)) < (__dIN_RANGE_TYPENAME__ _sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)((__dIN_RANGE_TYPENAME__ _sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)(amax) - (__dIN_RANGE_TYPENAME__ _sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)(amin)))
#define dCLAMP(aval, alo, ahi) ((aval) <= (alo) ? (alo) : (aval) >= (ahi) ? (ahi) : (aval))
#define dARRAY_SIZE(aarr) (sizeof(aarr) / sizeof((aarr)[0]))


#endif
