#ifndef _ODE_COMMON_INERNAL_H_
#define _ODE_COMMON_INERNAL_H_

#include <ode/common.h>


#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif



#include <math.h>


#ifdef HAVE_STDINT_H
#include <stdint.h>
#else
#ifdef HAVE_INTTYPES_H
#include <inttypes.h>
#else
#ifdef _MSC_VER
  typedef __int32           int32_t;
  typedef unsigned __int32  uint32_t;
  typedef __int16           int16_t;
  typedef unsigned __int16  uint16_t;
  typedef __int8            int8_t;
  typedef unsigned __int8   uint8_t;
#else
  typedef int             int32_t;
  typedef unsigned        uint32_t;
  typedef short           int16_t;
  typedef unsigned short  uint16_t;
  typedef signed char     int8_t;
  typedef unsigned char   uint8_t;
#endif
#endif
#endif


#include <stdlib.h>
/* An integer type that can be safely cast to a pointer. This definition
 * should be safe even on 64-bit systems */
typedef size_t intP;



/* configuration stuff */

/* the efficient alignment. most platforms align data structures to some
 * number of bytes, but this is not always the most efficient alignment.
 * for example, many x86 compilers align to 4 bytes, but on a pentium it
 * is important to align doubles to 8 byte boundaries (for speed), and
 * the 4 floats in a SIMD register to 16 byte boundaries. many other
 * platforms have similar behavior. setting a larger alignment can waste
 * a (very) small amount of memory. NOTE: this number must be a power of
 * two. this is set to 16 by default.
 */
#ifndef EFFICIENT_ALIGNMENT
#define EFFICIENT_ALIGNMENT 16
#endif

/* constants */

/* pi and 1/sqrt(2) are defined here if necessary because they don't get
 * defined in <math.h> on some platforms (like MS-Windows)
 */

#ifndef M_PI
#define M_PI REAL(3.1415926535897932384626433832795029)
#endif
#ifndef M_SQRT1_2
#define M_SQRT1_2 REAL(0.7071067811865475244008443621048490)
#endif



// Detect if we've got both trimesh engines enabled.
#ifdef dTRIMESH_ENABLED
#if defined(dTRIMESH_OPCODE) && defined(dTRIMESH_GIMPACT)
#error You can only #define dTRIMESH_OPCODE or dTRIMESH_GIMPACT, not both.
#endif
#endif // dTRIMESH_ENABLED




/* round an integer up to a multiple of 4, except that 0 and 1 are unmodified
 * (used to compute matrix leading dimensions)
 */
#define dPAD(a) (((a) > 1) ? ((((a)-1)|3)+1) : (a))






/* precision dependent scalar math functions */
#ifdef dSINGLE

#ifdef HAVE___ISNANF
#define dIsNan(x) (__isnanf(x))
#elif defined(HAVE__ISNANF)
#define dIsNan(x) (_isnanf(x))
#elif defined(HAVE_ISNANF)
#define dIsNan(x) (isnanf(x))
#else
  /*
     fall back to _isnan which is the VC way,
     this may seem redundant since we already checked
     for _isnan before, but if isnan is detected by
     configure but is not found during compilation
     we should always make sure we check for __isnanf,
     _isnanf and isnanf in that order before falling
     back to a default
  */
#define dIsNan(x) (_isnan(x))
#endif

#define dCopySign(a,b) ((dReal)copysignf(a,b))

#elif defined(dDOUBLE)

#ifdef HAVE___ISNAN
#define dIsNan(x) (__isnan(x))
#elif defined(HAVE__ISNAN)
#define dIsNan(x) (_isnan(x))
#elif defined(HAVE_ISNAN)
#define dIsNan(x) (isnan(x))
#else
#define dIsNan(x) (_isnan(x))
#endif

#define dCopySign(a,b) (copysign((a),(b)))

#else
#error You must #define dSINGLE or dDOUBLE
#endif







/* utility */


/* round something up to be a multiple of the EFFICIENT_ALIGNMENT */

#define dEFFICIENT_SIZE(x) ((((x)-1)|(EFFICIENT_ALIGNMENT-1))+1)


/* alloca aligned to the EFFICIENT_ALIGNMENT. note that this can waste
 * up to 15 bytes per allocation, depending on what alloca() returns.
 */
#ifdef HAVE_ALLOCA_H
#  include <alloca.h>
#else
#  include <malloc.h>
#  if defined(_MSC_VER) && !defined(alloca)
#    define alloca _alloca
#  endif
#endif

#define dALLOCA16(n) \
  ((char*)dEFFICIENT_SIZE(((size_t)(alloca((n)+(EFFICIENT_ALIGNMENT-1))))))


// Use the error-checking memory allocation system.  Because this system uses heap
//  (malloc) instead of stack (alloca), it is slower.  However, it allows you to
//  simulate larger scenes, as well as handle out-of-memory errors in a somewhat
//  graceful manner

// #define dUSE_MALLOC_FOR_ALLOCA

#ifdef dUSE_MALLOC_FOR_ALLOCA
enum {
  d_MEMORY_OK = 0,		/* no memory errors */
  d_MEMORY_OUT_OF_MEMORY	/* malloc failed due to out of memory error */
};

#endif




#endif

