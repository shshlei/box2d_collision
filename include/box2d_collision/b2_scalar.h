/*
Copyright (c) 2003-2013 Gino van den Bergen / Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*This file is copied from Bullet3 b3Scalar.h*/

#ifndef B2_SCALAR_H
#define B2_SCALAR_H

#ifdef B2_MANAGED_CODE
//Aligned data types not supported in managed code
#pragma unmanaged
#endif

#include <math.h>
#include <stdlib.h>  //size_t for MSVC 6.0
#include <float.h>

#if defined(DEBUG) || defined(_DEBUG)
    #define B2_DEBUG
#endif

#include "b2_logging.h" //for b2Error

#ifdef _WIN32
    #if  defined(__GNUC__)	// it should handle both MINGW and CYGWIN
        #define B2_FORCE_INLINE             __inline__ __attribute__((always_inline))
        #define B2_ATTRIBUTE_ALIGNED16(a)   a __attribute__((aligned(16)))
        #define B2_ATTRIBUTE_ALIGNED64(a)   a __attribute__((aligned(64)))
        #define B2_ATTRIBUTE_ALIGNED128(a)  a __attribute__((aligned(128)))
    #elif ( defined(_MSC_VER) && _MSC_VER < 1300 )
        #define B2_FORCE_INLINE inline
        #define B2_ATTRIBUTE_ALIGNED16(a) a
        #define B2_ATTRIBUTE_ALIGNED64(a) a
        #define B2_ATTRIBUTE_ALIGNED128(a) a
    #else //__MINGW32__
        //#define B2_HAS_ALIGNED_ALLOCATOR
        #pragma warning(disable : 4324)  // disable padding warning
        //#pragma warning(disable:4530) // Disable the exception disable but used in MSCV Stl warning.
        #pragma warning(disable : 4996)  //Turn off warnings about deprecated C routines
        //#pragma warning(disable:4786) // Disable the "debug name too long" warning
        #define B2_FORCE_INLINE __forceinline
        #define B2_ATTRIBUTE_ALIGNED16(a) __declspec(align(16)) a
        #define B2_ATTRIBUTE_ALIGNED64(a) __declspec(align(64)) a
        #define B2_ATTRIBUTE_ALIGNED128(a) __declspec(align(128)) a
        #ifdef _XBOX
            #define B2_USE_VMX128
            #include <ppcintrinsics.h>
            #define B2_HAVE_NATIVE_FSEL
            #define b2Fsel(a, b, c) __fsel((a), (b), (c))
        #else
            #if (defined(_WIN32) && (_MSC_VER) && _MSC_VER >= 1400) && (!defined(B2_USE_DOUBLE_PRECISION))
            #if (defined(_M_IX86) || defined(_M_X64))
                #ifdef __clang__
                    //#define B2_NO_SIMD_OPERATOR_OVERLOADS
                    #define B2_DISABLE_SSE
                #endif //__clang__
                #ifndef B2_DISABLE_SSE
                    #define B2_USE_SSE
                #endif //B2_DISABLE_SSE
                #ifdef B2_USE_SSE
                    //B2_USE_SSE_IN_API is disabled under Windows by default, because
                    //it makes it harder to integrate Bullet into your application under Windows
                    //(structured embedding Bullet structs/classes need to be 16-byte aligned)
                    //with relatively little performance gain
                    //If you are not embedded Bullet data in your classes, or make sure that you align those classes on 16-byte boundaries
                    //you can manually enable this line or set it in the build system for a bit of performance gain (a few percent, dependent on usage)
                    //#define B2_USE_SSE_IN_API
                #endif  //B2_USE_SSE
                #include <emmintrin.h>
            #endif
            #endif
        #endif  //_XBOX
    #endif  //__MINGW32__
    #ifdef B2_DEBUG
        #ifdef _MSC_VER
            #include <stdio.h>
            #define b2Assert(x) { if(!(x)){b2Error("Assert " __FILE__ ":%u (%s)\n", __LINE__, #x);__debugbreak();	}}
        #else  //_MSC_VER
            #include <assert.h>
            #define b2Assert assert
        #endif  //_MSC_VER
    #else
        #define b2Assert(x)
    #endif
    //b2FullAssert is optional, slows down a lot
    #define b2FullAssert(x)
    #define b2Likely(_c) _c
    #define b2Unlikely(_c) _c
#else //_WIN32
    #if defined(__CELLOS_LV2__)
        #define B2_FORCE_INLINE inline __attribute__((always_inline))
        #define B2_ATTRIBUTE_ALIGNED16(a) a __attribute__((aligned(16)))
        #define B2_ATTRIBUTE_ALIGNED64(a) a __attribute__((aligned(64)))
        #define B2_ATTRIBUTE_ALIGNED128(a) a __attribute__((aligned(128)))
        #ifndef assert
            #include <assert.h>
        #endif
        #ifdef B2_DEBUG
            #ifdef __SPU__
                #include <spu_printf.h>
                #define printf spu_printf
                #define b2Assert(x)               \
                    {                             \
                        if (!(x))                 \
                        {                         \
                            b2Error(              \
                                "Assert "__FILE__ \
                                ":%u (" #x ")\n", \
                                __LINE__);        \
                            spu_hcmpeq(0, 0);     \
                        }                         \
                    }
             #else
                #define b2Assert assert
            #endif
        #else
            #define b2Assert(x)
        #endif
        //b2FullAssert is optional, slows down a lot
        #define b2FullAssert(x)
        #define b2Likely(_c) _c
        #define b2Unlikely(_c) _c
    #else //__CELLOS_LV2__
        #ifdef USE_LIBSPE2
            #define B2_FORCE_INLINE __inline
            #define B2_ATTRIBUTE_ALIGNED16(a) a __attribute__((aligned(16)))
            #define B2_ATTRIBUTE_ALIGNED64(a) a __attribute__((aligned(64)))
            #define B2_ATTRIBUTE_ALIGNED128(a) a __attribute__((aligned(128)))
            #ifndef assert
                #include <assert.h>
            #endif
            #ifdef B2_DEBUG
                #define b2Assert assert
            #else
                #define b2Assert(x)
            #endif
            //b2FullAssert is optional, slows down a lot
            #define b2FullAssert(x)
            #define b2Likely(_c) __builtin_expect((_c), 1)
            #define b2Unlikely(_c) __builtin_expect((_c), 0)
        #else //LIBSPE2
            //non-windows systems
            #if (defined(__APPLE__) && (!defined(B2_USE_DOUBLE_PRECISION)))
                #if defined(__i386__) || defined(__x86_64__)
                    #define B2_USE_SSE
                    //B2_USE_SSE_IN_API is enabled on Mac OSX by default, because memory is automatically aligned on 16-byte boundaries
                    //if apps run into issues, we will disable the next line
                    #define B2_USE_SSE_IN_API
                    #ifdef B2_USE_SSE
                        // include appropriate SSE level
                        #if defined(__SSE4_1__)
                            #include <smmintrin.h>
                        #elif defined(__SSSE3__)
                            #include <tmmintrin.h>
                        #elif defined(__SSE3__)
                            #include <pmmintrin.h>
                        #else
                            #include <emmintrin.h>
                        #endif
                    #endif  //B2_USE_SSE
                #elif defined(__armv7__)
                    #ifdef __clang__
                        #define B2_USE_NEON 1
                        #if defined B2_USE_NEON && defined(__clang__)
                            #include <arm_neon.h>
                        #endif  //B2_USE_NEON
                    #endif  //__clang__
                #endif  //__arm__
                #define B2_FORCE_INLINE inline __attribute__((always_inline))
                ///@todo: check out alignment methods for other platforms/compilers
                #define B2_ATTRIBUTE_ALIGNED16(a) a __attribute__((aligned(16)))
                #define B2_ATTRIBUTE_ALIGNED64(a) a __attribute__((aligned(64)))
                #define B2_ATTRIBUTE_ALIGNED128(a) a __attribute__((aligned(128)))
                #ifndef assert
                    #include <assert.h>
                #endif
                #if defined(DEBUG) || defined(_DEBUG)
                    #if defined(__i386__) || defined(__x86_64__)
                        #include <stdio.h>
                        #define b2Assert(x)                                                             \
                        	{                                                                           \
                        		if (!(x))                                                               \
                        		{                                                                       \
                                    b2Error("Assert %s in line %d, file %s\n", #x, __LINE__, __FILE__); \
                                    asm volatile("int3");                                               \
                                }                                                                       \
                        	}
                    #else  //defined (__i386__) || defined (__x86_64__)
                        #define b2Assert assert
                    #endif  //defined (__i386__) || defined (__x86_64__)
                #else   //defined(DEBUG) || defined (_DEBUG)
                    #define b2Assert(x)
                #endif  //defined(DEBUG) || defined (_DEBUG)
                //b2FullAssert is optional, slows down a lot
                #define b2FullAssert(x)
                #define b2Likely(_c) _c
                #define b2Unlikely(_c) _c
            #else
                #define B2_FORCE_INLINE inline
                ///@todo: check out alignment methods for other platforms/compilers
                #define B2_ATTRIBUTE_ALIGNED16(a) a __attribute__((aligned(16)))
                #define B2_ATTRIBUTE_ALIGNED64(a) a __attribute__((aligned(64)))
                #define B2_ATTRIBUTE_ALIGNED128(a) a __attribute__((aligned(128)))
                ///#define B2_ATTRIBUTE_ALIGNED16(a) a
                ///#define B2_ATTRIBUTE_ALIGNED64(a) a
                ///#define B2_ATTRIBUTE_ALIGNED128(a) a
                #ifndef assert
                    #include <assert.h>
                #endif
                #if defined(DEBUG) || defined(_DEBUG)
                    #define b2Assert assert
                #else
                    #define b2Assert(x)
                #endif
                //b2FullAssert is optional, slows down a lot
                #define b2FullAssert(x)
                #define b2Likely(_c) _c
                #define b2Unlikely(_c) _c
            #endif  //__APPLE__
        #endif  // LIBSPE2
    #endif  //__CELLOS_LV2__
#endif//_WIN32

///The b2Scalar type abstracts floating point numbers, to easily switch between double and single floating point precision.
#if defined(B2_USE_DOUBLE_PRECISION)
    typedef double b2Scalar;
    //this number could be bigger in double precision
    #define B2_LARGE_FLOAT 1e30
#else
    typedef float b2Scalar;
    //keep B2_LARGE_FLOAT*B2_LARGE_FLOAT < FLT_MAX
    #define B2_LARGE_FLOAT 1e18f
#endif

#ifdef B2_USE_SSE
    typedef __m128 b2SimdFloat4;
#endif  //B2_USE_SSE

#if defined B2_USE_SSE_IN_API && defined(B2_USE_SSE)
    #ifdef _WIN32
        #ifndef B2_NAN
            static int b2NanMask = 0x7F800001;
            #define B2_NAN (*(float *)&b2NanMask)
        #endif
        #ifndef B2_INFINITY_MASK
            static int b2InfinityMask = 0x7F800000;
            #define B2_INFINITY_MASK (*(float *)&b2InfinityMask)
        #endif
        #ifndef B2_NO_SIMD_OPERATOR_OVERLOADS
            inline __m128 operator+(const __m128 A, const __m128 B)
            {
                return _mm_add_ps(A, B);
            }

            inline __m128 operator-(const __m128 A, const __m128 B)
            {
                return _mm_sub_ps(A, B);
            }

            inline __m128 operator*(const __m128 A, const __m128 B)
            {
                return _mm_mul_ps(A, B);
            }
        #endif //B2_NO_SIMD_OPERATOR_OVERLOADS
        #define b2CastfTo128i(a) (_mm_castps_si128(a))
        #define b2CastfTo128d(a) (_mm_castps_pd(a))
        #define b2CastiTo128f(a) (_mm_castsi128_ps(a))
        #define b2CastdTo128f(a) (_mm_castpd_ps(a))
        #define b2CastdTo128i(a) (_mm_castpd_si128(a))
        #define b2Assign128(r0, r1, r2, r3) _mm_setr_ps(r0, r1, r2, r3)
    #else  //_WIN32
        #define b2CastfTo128i(a) ((__m128i)(a))
        #define b2CastfTo128d(a) ((__m128d)(a))
        #define b2CastiTo128f(a) ((__m128)(a))
        #define b2CastdTo128f(a) ((__m128)(a))
        #define b2CastdTo128i(a) ((__m128i)(a))
        #define b2Assign128(r0, r1, r2, r3) \
        	(__m128) { r0, r1, r2, r3 }
    #endif  //_WIN32
#endif  //B2_USE_SSE_IN_API

#ifdef B2_USE_NEON
    #include <arm_neon.h>
    typedef float32x4_t b2SimdFloat4;
    #define B2_INFINITY INFINITY
    #define B2_NAN NAN
    #define b2Assign128(r0, r1, r2, r3) \
    	(float32x4_t) { r0, r1, r2, r3 }
#endif

#define B2_DECLARE_ALIGNED_ALLOCATOR()                                                                   \
	B2_FORCE_INLINE void *operator new(size_t sizeInBytes) { return b2AlignedAlloc(sizeInBytes, 16); }   \
	B2_FORCE_INLINE void operator delete(void *ptr) { b2AlignedFree(ptr); }                              \
	B2_FORCE_INLINE void *operator new(size_t, void *ptr) { return ptr; }                                \
	B2_FORCE_INLINE void operator delete(void *, void *) {}                                              \
	B2_FORCE_INLINE void *operator new[](size_t sizeInBytes) { return b2AlignedAlloc(sizeInBytes, 16); } \
	B2_FORCE_INLINE void operator delete[](void *ptr) { b2AlignedFree(ptr); }                            \
	B2_FORCE_INLINE void *operator new[](size_t, void *ptr) { return ptr; }                              \
	B2_FORCE_INLINE void operator delete[](void *, void *) {}

#if defined(B2_USE_DOUBLE_PRECISION) || defined(B2_FORCE_DOUBLE_FUNCTIONS)
    B2_FORCE_INLINE b2Scalar b2Sqrt(b2Scalar x)
    {
        return sqrt(x);
    }
    B2_FORCE_INLINE b2Scalar b2Fabs(b2Scalar x) { return fabs(x); }
    B2_FORCE_INLINE b2Scalar b2Cos(b2Scalar x) { return cos(x); }
    B2_FORCE_INLINE b2Scalar b2Sin(b2Scalar x) { return sin(x); }
    B2_FORCE_INLINE b2Scalar b2Tan(b2Scalar x) { return tan(x); }
    B2_FORCE_INLINE b2Scalar b2Acos(b2Scalar x)
    {
        if (x < b2Scalar(-1)) x = b2Scalar(-1);
        if (x > b2Scalar(1)) x = b2Scalar(1);
        return acos(x);
    }
    B2_FORCE_INLINE b2Scalar b2Asin(b2Scalar x)
    {
        if (x < b2Scalar(-1)) x = b2Scalar(-1);
        if (x > b2Scalar(1)) x = b2Scalar(1);
        return asin(x);
    }
    B2_FORCE_INLINE b2Scalar b2Atan(b2Scalar x) { return atan(x); }
    B2_FORCE_INLINE b2Scalar b2Atan2(b2Scalar x, b2Scalar y) { return atan2(x, y); }
    B2_FORCE_INLINE b2Scalar b2Exp(b2Scalar x) { return exp(x); }
    B2_FORCE_INLINE b2Scalar b2Log(b2Scalar x) { return log(x); }
    B2_FORCE_INLINE b2Scalar b2Pow(b2Scalar x, b2Scalar y) { return pow(x, y); }
    B2_FORCE_INLINE b2Scalar b2Fmod(b2Scalar x, b2Scalar y) { return fmod(x, y); }
#else
    B2_FORCE_INLINE b2Scalar b2Sqrt(b2Scalar y)
    {
#ifdef USE_APPROXIMATION
        double x, z, tempf;
        unsigned long *tfptr = ((unsigned long *)&tempf) + 1;

        tempf = y;
        *tfptr = (0xbfcdd90a - *tfptr) >> 1; /* estimate of 1/sqrt(y) */
        x = tempf;
        z = y * b2Scalar(0.5);
        x = (b2Scalar(1.5) * x) - (x * x) * (x * z); /* iteration formula     */
        x = (b2Scalar(1.5) * x) - (x * x) * (x * z);
        x = (b2Scalar(1.5) * x) - (x * x) * (x * z);
        x = (b2Scalar(1.5) * x) - (x * x) * (x * z);
        x = (b2Scalar(1.5) * x) - (x * x) * (x * z);
        return x * y;
#else
        return sqrtf(y);
#endif
    }
    B2_FORCE_INLINE b2Scalar b2Fabs(b2Scalar x) { return fabsf(x); }
    B2_FORCE_INLINE b2Scalar b2Cos(b2Scalar x) { return cosf(x); }
    B2_FORCE_INLINE b2Scalar b2Sin(b2Scalar x) { return sinf(x); }
    B2_FORCE_INLINE b2Scalar b2Tan(b2Scalar x) { return tanf(x); }
    B2_FORCE_INLINE b2Scalar b2Acos(b2Scalar x)
    {
        if (x < b2Scalar(-1))
            x = b2Scalar(-1);
        if (x > b2Scalar(1))
            x = b2Scalar(1);
        return acosf(x);
    }
    B2_FORCE_INLINE b2Scalar b2Asin(b2Scalar x)
    {
        if (x < b2Scalar(-1))
            x = b2Scalar(-1);
        if (x > b2Scalar(1))
            x = b2Scalar(1);
        return asinf(x);
    }
    B2_FORCE_INLINE b2Scalar b2Atan(b2Scalar x) { return atanf(x); }
    B2_FORCE_INLINE b2Scalar b2Atan2(b2Scalar x, b2Scalar y) { return atan2f(x, y); }
    B2_FORCE_INLINE b2Scalar b2Exp(b2Scalar x) { return expf(x); }
    B2_FORCE_INLINE b2Scalar b2Log(b2Scalar x) { return logf(x); }
    B2_FORCE_INLINE b2Scalar b2Pow(b2Scalar x, b2Scalar y) { return powf(x, y); }
    B2_FORCE_INLINE b2Scalar b2Fmod(b2Scalar x, b2Scalar y) { return fmodf(x, y); }
#endif

#define B2_2_PI b2Scalar(6.283185307179586232)
#define B2_PI (B2_2_PI * b2Scalar(0.5))
#define B2_HALF_PI (B2_2_PI * b2Scalar(0.25))
#define B2_RADS_PER_DEG (B2_2_PI / b2Scalar(360.0))
#define B2_DEGS_PER_RAD (b2Scalar(360.0) / B2_2_PI)
#define B2_SQRT12 b2Scalar(0.7071067811865475244008443621048490)

#define b2RecipSqrt(x) ((b2Scalar)(b2Scalar(1.0) / b2Sqrt(b2Scalar(x)))) /* reciprocal square root */

#ifdef B2_USE_DOUBLE_PRECISION
    #define B2_EPSILON DBL_EPSILON
    #define B2_INFINITY DBL_MAX
#else
    #define B2_EPSILON FLT_EPSILON
    #define B2_INFINITY FLT_MAX
#endif

B2_FORCE_INLINE b2Scalar b2Atan2Fast(b2Scalar y, b2Scalar x)
{
	b2Scalar coeff_1 = B2_PI / 4.0f;
	b2Scalar coeff_2 = 3.0f * coeff_1;
	b2Scalar abs_y = b2Fabs(y);
	b2Scalar angle;
	if (x >= 0.0f)
	{
		b2Scalar r = (x - abs_y) / (x + abs_y);
		angle = coeff_1 - coeff_1 * r;
	}
	else
	{
		b2Scalar r = (x + abs_y) / (abs_y - x);
		angle = coeff_2 - coeff_1 * r;
	}
	return (y < 0.0f) ? -angle : angle;
}

B2_FORCE_INLINE bool b2FuzzyZero(b2Scalar x) { return b2Fabs(x) < B2_EPSILON; }

B2_FORCE_INLINE bool b2Equal(b2Scalar a, b2Scalar eps)
{
	return (((a) <= eps) && !((a) < -eps));
}
B2_FORCE_INLINE bool b2GreaterEqual(b2Scalar a, b2Scalar eps)
{
	return (!((a) <= eps));
}

B2_FORCE_INLINE int b2IsNegative(b2Scalar x)
{
	return x < b2Scalar(0.0) ? 1 : 0;
}

B2_FORCE_INLINE b2Scalar b2Radians(b2Scalar x) { return x * B2_RADS_PER_DEG; }
B2_FORCE_INLINE b2Scalar b2Degrees(b2Scalar x) { return x * B2_DEGS_PER_RAD; }

#define B2_DECLARE_HANDLE(name) \
	typedef struct name##__     \
	{                           \
		int unused;             \
	} * name

#ifndef b2Fsel
    B2_FORCE_INLINE b2Scalar b2Fsel(b2Scalar a, b2Scalar b, b2Scalar c)
    {
        return a >= 0 ? b : c;
    }
#endif
#define b2Fsels(a, b, c) (b2Scalar) b2Fsel(a, b, c)

B2_FORCE_INLINE bool b2MachineIsLittleEndian()
{
	long int i = 1;
	const char *p = (const char *)&i;
	if (p[0] == 1)  // Lowest address contains the least significant byte
		return true;
	else
		return false;
}

///b2Select avoids branches, which makes performance much better for consoles like Playstation 3 and XBox 360
///Thanks Phil Knight. See also http://www.cellperformance.com/articles/2006/04/more_techniques_for_eliminatin_1.html
B2_FORCE_INLINE unsigned b2Select(unsigned condition, unsigned valueIfConditionNonZero, unsigned valueIfConditionZero)
{
	// Set testNz to 0xFFFFFFFF if condition is nonzero, 0x00000000 if condition is zero
	// Rely on positive value or'ed with its negative having sign bit on
	// and zero value or'ed with its negative (which is still zero) having sign bit off
	// Use arithmetic shift right, shifting the sign bit through all 32 bits
	unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
	unsigned testEqz = ~testNz;
	return ((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz));
}
B2_FORCE_INLINE int b2Select(unsigned condition, int valueIfConditionNonZero, int valueIfConditionZero)
{
	unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
	unsigned testEqz = ~testNz;
	return static_cast<int>((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz));
}
B2_FORCE_INLINE float b2Select(unsigned condition, float valueIfConditionNonZero, float valueIfConditionZero)
{
#ifdef B2_HAVE_NATIVE_FSEL
	return (float)b2Fsel((b2Scalar)condition - b2Scalar(1.0f), valueIfConditionNonZero, valueIfConditionZero);
#else
	return (condition != 0) ? valueIfConditionNonZero : valueIfConditionZero;
#endif
}

template <typename T>
B2_FORCE_INLINE void b2Swap(T &a, T &b)
{
	T tmp = a;
	a = b;
	b = tmp;
}

//PCK: endian swapping functions
B2_FORCE_INLINE unsigned b2SwapEndian(unsigned val)
{
	return (((val & 0xff000000) >> 24) | ((val & 0x00ff0000) >> 8) | ((val & 0x0000ff00) << 8) | ((val & 0x000000ff) << 24));
}

B2_FORCE_INLINE unsigned short b2SwapEndian(unsigned short val)
{
	return static_cast<unsigned short>(((val & 0xff00) >> 8) | ((val & 0x00ff) << 8));
}

B2_FORCE_INLINE unsigned b2SwapEndian(int val)
{
	return b2SwapEndian((unsigned)val);
}

B2_FORCE_INLINE unsigned short b2SwapEndian(short val)
{
	return b2SwapEndian((unsigned short)val);
}

///b2SwapFloat uses using char pointers to swap the endianness
////b2SwapFloat/b2SwapDouble will NOT return a float, because the machine might 'correct' invalid floating point values
///Not all values of sign/exponent/mantissa are valid floating point numbers according to IEEE 754.
///When a floating point unit is faced with an invalid value, it may actually change the value, or worse, throw an exception.
///In most systems, running user mode code, you wouldn't get an exception, but instead the hardware/os/runtime will 'fix' the number for you.
///so instead of returning a float/double, we return integer/long long integer
B2_FORCE_INLINE unsigned int b2SwapEndianFloat(float d)
{
	unsigned int a = 0;
	unsigned char *dst = (unsigned char *)&a;
	unsigned char *src = (unsigned char *)&d;

	dst[0] = src[3];
	dst[1] = src[2];
	dst[2] = src[1];
	dst[3] = src[0];
	return a;
}

// unswap using char pointers
B2_FORCE_INLINE float b2UnswapEndianFloat(unsigned int a)
{
	float d = 0.0f;
	unsigned char *src = (unsigned char *)&a;
	unsigned char *dst = (unsigned char *)&d;

	dst[0] = src[3];
	dst[1] = src[2];
	dst[2] = src[1];
	dst[3] = src[0];

	return d;
}

// swap using char pointers
B2_FORCE_INLINE void b2SwapEndianDouble(double d, unsigned char *dst)
{
	unsigned char *src = (unsigned char *)&d;

	dst[0] = src[7];
	dst[1] = src[6];
	dst[2] = src[5];
	dst[3] = src[4];
	dst[4] = src[3];
	dst[5] = src[2];
	dst[6] = src[1];
	dst[7] = src[0];
}

// unswap using char pointers
B2_FORCE_INLINE double b2UnswapEndianDouble(const unsigned char *src)
{
	double d = 0.0;
	unsigned char *dst = (unsigned char *)&d;

	dst[0] = src[7];
	dst[1] = src[6];
	dst[2] = src[5];
	dst[3] = src[4];
	dst[4] = src[3];
	dst[5] = src[2];
	dst[6] = src[1];
	dst[7] = src[0];

	return d;
}

// returns normalized value in range [-B2_PI, B2_PI]
B2_FORCE_INLINE b2Scalar b2NormalizeAngle(b2Scalar angleInRadians)
{
	angleInRadians = b2Fmod(angleInRadians, B2_2_PI);
	if (angleInRadians < -B2_PI)
	{
		return angleInRadians + B2_2_PI;
	}
	else if (angleInRadians > B2_PI)
	{
		return angleInRadians - B2_2_PI;
	}
	else
	{
		return angleInRadians;
	}
}

///rudimentary class to provide type info
struct b2TypedObject
{
	b2TypedObject(int objectType)
		: m_objectType(objectType)
	{
	}
	int m_objectType;
	inline int getObjectType() const
	{
		return m_objectType;
	}
};

///align a pointer to the provided alignment, upwards
template <typename T>
T *b2AlignPointer(T *unalignedPtr, size_t alignment)
{
	struct b2ConvertPointerSizeT
	{
		union {
			T *ptr;
			size_t integer;
		};
	};
	b2ConvertPointerSizeT converter;

	const size_t bit_mask = ~(alignment - 1);
	converter.ptr = unalignedPtr;
	converter.integer += alignment - 1;
	converter.integer &= bit_mask;
	return converter.ptr;
}

#endif  //B2_SCALAR_H
