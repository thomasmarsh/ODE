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


#ifndef offsetof
#define offsetof(s, m) ((size_t)&(((s *)8)->m) - (size_t)8)
#endif
#ifndef membersize
#define membersize(s, m) (sizeof(((s *)8)->m))
#endif
#ifndef endoffsetof
#define endoffsetof(s, m)   ((size_t)((size_t)&(((s *)8)->m) - (size_t)8) + sizeof(((s *)8)->m))
#endif

#define dMACRO_MAX(a, b) ((a) > (b) ? (a) : (b))
#define dMACRO_MIN(a, b) ((a) < (b) ? (a) : (b))

#define dMAKE_PADDING_SIZE(DataType, ElementType) ((sizeof(DataType) + sizeof(ElementType) - 1) / sizeof(ElementType))


template<typename DstType, typename SrcType>
inline 
bool _cast_to_smaller(DstType &dtOutResult, const SrcType &stArgument)
{
    return (SrcType)(dtOutResult = (DstType)stArgument) == stArgument;
}

#if defined(__GNUC__)

#define dCAST_TO_SMALLER(TargetType, SourceValue) ({ TargetType ttCastSmallerValue; dIVERIFY(_cast_to_smaller(ttCastSmallerValue, SourceValue)); ttCastSmallerValue; })


#else // #if !defined(__GNUC__)

#define dCAST_TO_SMALLER(TargetType, SourceValue) templateCAST_TO_SMALLER<TargetType>(SourceValue)

template <typename TTargetType, typename TSourceType>
inline TTargetType templateCAST_TO_SMALLER(const TSourceType &stSourceValue)
{
    TTargetType ttCastSmallerValue;
    dIVERIFY(_cast_to_smaller(ttCastSmallerValue, stSourceValue));
    return ttCastSmallerValue;
}


#endif // #if !defined(__GNUC__)



template <typename Type>
union _const_type_cast_union
{
    explicit _const_type_cast_union(const void *psvCharBuffer): m_psvCharBuffer(psvCharBuffer) {}

    operator const Type *() const { return m_pstTypedPointer; }
    const Type &operator *() const { return *m_pstTypedPointer; }
    const Type *operator ->() const { return m_pstTypedPointer; }
    const Type &operator [](ptrdiff_t diElementIndex) const { return m_pstTypedPointer[diElementIndex]; }
    const Type &operator [](size_t siElementIndex) const { return m_pstTypedPointer[siElementIndex]; }

    const void 		*m_psvCharBuffer;
    const Type		*m_pstTypedPointer;
};

template <typename Type>
union _type_cast_union
{
    explicit _type_cast_union(void *psvCharBuffer): m_psvCharBuffer(psvCharBuffer) {}

    operator Type *() const { return m_pstTypedPointer; }
    Type &operator *() const { return *m_pstTypedPointer; }
    Type *operator ->() const { return m_pstTypedPointer; }
    Type &operator [](ptrdiff_t diElementIndex) const { return m_pstTypedPointer[diElementIndex]; }
    Type &operator [](size_t siElementIndex) const { return m_pstTypedPointer[siElementIndex]; }

    void			*m_psvCharBuffer;
    Type			*m_pstTypedPointer;
};


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
