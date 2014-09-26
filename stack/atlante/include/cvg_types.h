/**
 * Description: Definition of basic framework types
 * Note: The POSH library is used
 * Author: Ignacio Mellado (CVG)
 * Last revision date: 15/8/2011
 */

#ifndef CVG_TYPES_INCLUDED__
#define CVG_TYPES_INCLUDED__

#include "../config.h"
#include <limits.h>

/* If any type requires more than 32 bits, enable 64 bits support in POSH */
#if (CVG_REQUIRED_SIZEOF_CHAR > 4 || CVG_REQUIRED_SIZEOF_SHORT > 4 || CVG_REQUIRED_SIZEOF_INT > 4 || CVG_REQUIRED_SIZEOF_LONG > 4)
#define POSH_64BIT_INTEGER
#endif

/* Include POSH library for fixed size types and other functionalities */
#define POSH_USE_LIMITS_H	/* Better use standard C limits.h */
#ifdef CVG_DO_NOT_USE_FLOATING_POINT
#define POSH_NO_FLOAT
#endif
#include "../3rdparty/poshlib/posh.h"

/* Define this constant for any Windows system */
#if (POSH_OS_WIN32 || POSH_OS_WIN64 || POSH_OS_WINCE)
#define CVG_WINDOWS
#elif POSH_OS_CYGWIN32
#define CVG_CYGWIN
#endif

/* The framework boolean type is like the platform's */
typedef bool	cvg_bool;

/* Assign framework types to platform types. 
Assign the corresponding native type if it does the job. Otherwise, choose the 
smallest possible from the fixed size set */
/* char */
#if ( CVG_REQUIRED_MAX_UCHAR <= UCHAR_MAX )
	typedef char			cvg_char;
	typedef unsigned char	cvg_uchar;
	#define CVG_CHAR_MIN	CHAR_MIN
	#define CVG_CHAR_MAX	CHAR_MAX
	#define CVG_UCHAR_MIN	UCHAR_MIN
	#define CVG_UCHAR_MAX	UCHAR_MAX
#elif ( CVG_REQUIRED_MAX_UCHAR <= POSH_U16_MAX )
	typedef posh_i16_t		cvg_char;
	typedef posh_u16_t		cvg_uchar;
	#define CVG_CHAR_MIN	POSH_I16_MIN
	#define CVG_CHAR_MAX	POSH_I16_MAX
	#define CVG_UCHAR_MIN	POSH_U16_MIN
	#define CVG_UCHAR_MAX	POSH_U16_MAX
#elif ( CVG_REQUIRED_MAX_UCHAR <= POSH_U32_MAX )
	typedef posh_i32_t		cvg_char;
	typedef posh_u32_t		cvg_uchar;
	#define CVG_CHAR_MIN	POSH_I32_MIN
	#define CVG_CHAR_MAX	POSH_I32_MAX
	#define CVG_UCHAR_MIN	POSH_U32_MIN
	#define CVG_UCHAR_MAX	POSH_U32_MAX
#elif ( CVG_REQUIRED_MAX_UCHAR <= POSH_U64_MAX )
	typedef posh_i64_t		cvg_char;
	typedef posh_u64_t		cvg_uchar;
	#define CVG_CHAR_MIN	POSH_I64_MIN
	#define CVG_CHAR_MAX	POSH_I64_MAX
	#define CVG_UCHAR_MIN	POSH_U64_MIN
	#define CVG_UCHAR_MAX	POSH_U64_MAX
#else
	#error "The required value range for type cvg_char/cvg_uchar is too big"
#endif

/* short */
#if ( CVG_REQUIRED_MAX_USHORT <= USHRT_MAX )
	typedef short			cvg_short;
	typedef unsigned short	cvg_ushort;
	#define CVG_SHORT_MIN	SHRT_MIN
	#define CVG_SHORT_MAX	SHRT_MAX
	#define CVG_USHORT_MIN	USHRT_MIN
	#define CVG_USHORT_MAX	USHRT_MAX
#elif ( CVG_REQUIRED_MAX_USHORT <= POSH_U32_MAX )
	typedef posh_i32_t		cvg_short;
	typedef posh_u32_t		cvg_ushort;
	#define CVG_SHORT_MIN	POSH_I32_MIN
	#define CVG_SHORT_MAX	POSH_I32_MAX
	#define CVG_USHORT_MIN	POSH_U32_MIN
	#define CVG_USHORT_MAX	POSH_U32_MAX
#elif ( CVG_REQUIRED_MAX_USHORT <= POSH_U64_MAX )
	typedef posh_i64_t		cvg_short;
	typedef posh_u64_t		cvg_ushort;
	#define CVG_SHORT_MIN	POSH_I64_MIN
	#define CVG_SHORT_MAX	POSH_I64_MAX
	#define CVG_USHORT_MIN	POSH_U64_MIN
	#define CVG_USHORT_MAX	POSH_U64_MAX
#else
	#error "The required value range for type cvg_short/cvg_ushort is too big"
#endif

/* int */
#if ( CVG_REQUIRED_MAX_UINT <= UINT_MAX )
	typedef int				cvg_int;
	typedef unsigned int	cvg_uint;
	#define CVG_INT_MIN		INT_MIN
	#define CVG_INT_MAX		INT_MAX
	#define CVG_UINT_MIN	UINT_MIN
	#define CVG_UINT_MAX	UINT_MAX
#elif ( CVG_REQUIRED_MAX_UINT <= POSH_U64_MAX )
	typedef posh_i64_t		cvg_int;
	typedef posh_u64_t		cvg_uint;
	#define CVG_INT_MIN		POSH_I64_MIN
	#define CVG_INT_MAX		POSH_I64_MAX
	#define CVG_UINT_MIN	POSH_U64_MIN
	#define CVG_UINT_MAX	POSH_U64_MAX
#else
	#error "The required value range for type cvg_int/cvg_uint is too big"
#endif

/* long */
#if ( CVG_REQUIRED_MAX_ULONG <= ULONG_MAX )
	typedef long			cvg_long;
	typedef unsigned long	cvg_ulong;
	#define CVG_LONG_MIN	LONG_MIN
	#define CVG_LONG_MAX	LONG_MAX
	#define CVG_ULONG_MIN	ULONG_MIN
	#define CVG_ULONG_MAX	ULONG_MAX
#elif ( CVG_REQUIRED_MAX_ULONG <= POSH_U64_MAX )
	typedef posh_i64_t		cvg_long;
	typedef posh_u64_t		cvg_ulong;
	#define CVG_LONG_MIN	POSH_I64_MIN
	#define CVG_LONG_MAX	POSH_I64_MAX
	#define CVG_ULONG_MIN	POSH_U64_MIN
	#define CVG_ULONG_MAX	POSH_U64_MAX
#else
	#error "The required value range for type cvg_long/cvg_ulong is too big"
#endif

typedef float cvg_float;
typedef double cvg_double;

/* Macros for literal definition and printf prefixes */
/* Each type is only affected if it is assigned to a platform 64 bit type */

/* char */
#if ( CVG_REQUIRED_MAX_UCHAR >= POSH_U64_MAX )
	#define CVG_LITERAL_CHAR(x)			POSH_I64(x)
	#define CVG_LITERAL_UCHAR(x)		POSH_U64(x)
	#define CVG_PRINTF_PREFIX_CHAR		POSH_I64_PRINTF_PREFIX
#else
	#define CVG_LITERAL_CHAR(x)			(x)
	#define CVG_LITERAL_UCHAR(x)		(x)
	#define CVG_PRINTF_PREFIX_CHAR
#endif

/* short */
#if ( CVG_REQUIRED_MAX_USHORT >= POSH_U64_MAX )
	#define CVG_LITERAL_SHORT(x)		POSH_I64(x)
	#define CVG_LITERAL_USHORT(x)		POSH_U64(x)
	#define CVG_PRINTF_PREFIX_SHORT		POSH_I64_PRINTF_PREFIX
#else
	#define CVG_LITERAL_SHORT(x)		(x)
	#define CVG_LITERAL_USHORT(x)		(x)
	#define CVG_PRINTF_PREFIX_SHORT
#endif

/* int */
#if ( CVG_REQUIRED_MAX_UINT >= POSH_U64_MAX )
	#define CVG_LITERAL_INT(x)			POSH_I64(x)
	#define CVG_LITERAL_UINT(x)			POSH_U64(x)
	#define CVG_PRINTF_PREFIX_INT		POSH_I64_PRINTF_PREFIX
#else
	#define CVG_LITERAL_INT(x)			(x)
	#define CVG_LITERAL_UINT(x)			(x)
	#define CVG_PRINTF_PREFIX_INT
#endif

/* long */
#if ( CVG_REQUIRED_MAX_ULONG >= POSH_U64_MAX )
	#define CVG_LITERAL_LONG(x)			POSH_I64(x)
	#define CVG_LITERAL_ULONG(x)		POSH_U64(x)
	#define CVG_PRINTF_PREFIX_LONG		POSH_I64_PRINTF_PREFIX
#else
	#define CVG_LITERAL_LONG(x)			(x)
	#define CVG_LITERAL_ULONG(x)		(x)
	#define CVG_PRINTF_PREFIX_LONG
#endif

#define CVG_PRINTF_PREFIX_UCHAR		CVG_PRINTF_PREFIX_CHAR
#define CVG_PRINTF_PREFIX_USHORT	CVG_PRINTF_PREFIX_SHORT
#define CVG_PRINTF_PREFIX_UINT		CVG_PRINTF_PREFIX_INT
#define CVG_PRINTF_PREFIX_ULONG		CVG_PRINTF_PREFIX_LONG

/* Kill temporary definitions */
#undef POSH_USE_LIMITS_H
#undef POSH_64BIT_INTEGER
#undef POSH_NO_FLOAT

#endif	/* CVG_TYPES_INCLUDED__ */
