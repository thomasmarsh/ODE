/*************************************************************************
 * A custom config.h for the Premake contribution. I'm trying to see if
 * I can come up with a way to build the configuration via Premake,
 * instead of using the configurator executable. So far, I haven't needed
 * to generate anything, but I haven't attempted the PENTIUM flag yet.
 *************************************************************************/

#ifndef _ODE_CONFIG_H_
#define _ODE_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Try to identify the platform */
#if defined(_MSC_VER) || defined(__CYGWIN32__) || defined(__MINGW32__)
	#define ODE_PLATFORM_WIN32
#elif defined(__linux__)
	#define ODE_PLATFORM_LINUX
#elif defined(__APPLE__) && defined(__MACH__)
	#define ODE_PLATFORM_OSX
#else
	#error "Need some help identifying the platform!"
#endif

/* Additional platform defines used in the code */
#if defined(ODE_PLATFORM_WIN32) && !defined(WIN32)
	#define WIN32
#endif

#if defined(__CYGWIN32__) || defined(__MINGW32__)
	#define CYGWIN
#endif

#if defined(ODE_PLATFORM_OSX)
	#define macintosh
#endif


/* Pull in the standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <malloc.h>
#include <math.h>
#include <string.h>

#if !defined(ODE_PLATFORM_WIN32)
#include <alloca.h>
#endif


/* Visual C does not define these functions */
#if defined(_MSC_VER)
	#define copysignf _copysign
	#define copysign _copysign
#endif


/* Define a value for infinity */
#if defined(HUGE_VALF)
	#define ODE_INFINITY4 HUGE_VALF
	#define ODE_INFINITY8 HUGE_VAL
#elif defined(FLT_MAX)
	#define ODE_INFINITY4 FLT_MAX
	#define ODE_INFINITY8 DBL_MAX
#else
	static union { unsigned char __c[4]; float  __f; }  __ode_huge_valf = {{0,0,0x80,0x7f}};
	static union { unsigned char __c[8]; double __d; }  __ode_huge_val  = {{0,0,0,0,0,0,0xf0,0x7f}};
	#define ODE_INFINITY4 (__ode_huge_valf.__f)
	#define ODE_INFINITY8 (__ode_huge_val.__d)
#endif


/* Setup the desired precision */
#if defined(dSINGLE)
	#define dInfinity ODE_INFINITY4
	#define dEpsilon FLT_EPSILON
#else
	#define dInfinity ODE_INFINITY8
	#define dEpsilon DBL_EPSILON
#endif


/* Well-defined common data types...need to define for 64 bit system s*/
#if defined(_M_IA64) || defined(__ia64__) || defined(_M_AMD64) || defined(__x86_64__)
	#error "Define data types for 64-bit!"
#else
	typedef int             int32;
	typedef unsigned int    uint32;
	typedef short           int16;
	typedef unsigned short  uint16;
	typedef char            int8;
	typedef unsigned char   uint8;
#endif

/* An integer type that can be safely cast to a pointer. This definition
 * should be safe even on 64-bit systems */
typedef size_t intP;


/* The efficient alignment. most platforms align data structures to some
 * number of bytes, but this is not always the most efficient alignment.
 * for example, many x86 compilers align to 4 bytes, but on a pentium it is
 * important to align doubles to 8 byte boundaries (for speed), and the 4
 * floats in a SIMD register to 16 byte boundaries. many other platforms have
 * similar behavior. setting a larger alignment can waste a (very) small
 * amount of memory. NOTE: this number must be a power of two. */
#define EFFICIENT_ALIGNMENT 16


/* Define this if your system supports anonymous memory maps (linux does) */
#define MMAP_ANONYMOUS


#ifdef __cplusplus
}
#endif

#endif
