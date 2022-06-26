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

#ifndef B2_VECTOR3_H
#define B2_VECTOR3_H

#include "b2Scalar.h"
#include "b2MinMax.h"
#include "b2AlignedAllocator.h"

#ifdef B2_USE_DOUBLE_PRECISION
    #define b2Vector3Data b2Vector3DoubleData
    #define b2Vector3DataName "b2Vector3DoubleData"
#else
    #define b2Vector3Data b2Vector3FloatData
    #define b2Vector3DataName "b2Vector3FloatData"
#endif  //B2_USE_DOUBLE_PRECISION

#if defined B2_USE_SSE
    //typedef  uint32_t __m128i __attribute__ ((vector_size(16)));
    #ifdef _MSC_VER
        #pragma warning(disable : 4556)  // value of intrinsic immediate argument '4294967239' is out of range '0 - 255'
    #endif

    #define B2_SHUFFLE(x, y, z, w) (((w) << 6 | (z) << 4 | (y) << 2 | (x)) & 0xff)
    //#define B2_pshufd_ps( _a, _mask ) (__m128) _mm_shuffle_epi32((__m128i)(_a), (_mask) )
    #define B2_pshufd_ps(_a, _mask) _mm_shuffle_ps((_a), (_a), (_mask))
    #define B2_splat3_ps(_a, _i) B2_pshufd_ps((_a), B2_SHUFFLE(_i, _i, _i, 3))
    #define B2_splat_ps(_a, _i) B2_pshufd_ps((_a), B2_SHUFFLE(_i, _i, _i, _i))

    #define b2v3AbsiMask (_mm_set_epi32(0x00000000, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF))
    #define b2vAbsMask (_mm_set_epi32(0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF))
    #define b2vFFF0Mask (_mm_set_epi32(0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF))
    #define b2v3AbsfMask b2CastiTo128f(b2v3AbsiMask)
    #define b2vFFF0fMask b2CastiTo128f(b2vFFF0Mask)
    #define b2vxyzMaskf b2vFFF0fMask
    #define b2vAbsfMask b2CastiTo128f(b2vAbsMask)

    const __m128 B2_ATTRIBUTE_ALIGNED16(b2vMzeroMask) = {-0.0f, -0.0f, -0.0f, -0.0f};
    const __m128 B2_ATTRIBUTE_ALIGNED16(b2v1110) = {1.0f, 1.0f, 1.0f, 0.0f};
    const __m128 B2_ATTRIBUTE_ALIGNED16(b2vHalf) = {0.5f, 0.5f, 0.5f, 0.5f};
    const __m128 B2_ATTRIBUTE_ALIGNED16(b2v1_5) = {1.5f, 1.5f, 1.5f, 1.5f};
#endif

#ifdef B2_USE_NEON
    const float32x4_t B2_ATTRIBUTE_ALIGNED16(b2vMzeroMask) = (float32x4_t){-0.0f, -0.0f, -0.0f, -0.0f};
    const int32x4_t B2_ATTRIBUTE_ALIGNED16(b2vFFF0Mask) = (int32x4_t){0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x0};
    const int32x4_t B2_ATTRIBUTE_ALIGNED16(b2vAbsMask) = (int32x4_t){0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF};
    const int32x4_t B2_ATTRIBUTE_ALIGNED16(b2v3AbsMask) = (int32x4_t){0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x0};
#endif

class b2Vector3;
class b2Vector4;

#if defined(B3_USE_SSE_IN_API) && defined(B3_USE_SSE)
//#if defined (B3_USE_SSE) || defined (B3_USE_NEON)
inline b3Vector3 b3MakeVector3(b3SimdFloat4 v);
inline b3Vector4 b3MakeVector4(b3SimdFloat4 vec);
#endif

inline b3Vector3 b3MakeVector3(b3Scalar x, b3Scalar y, b3Scalar z);
inline b3Vector3 b3MakeVector3(b3Scalar x, b3Scalar y, b3Scalar z, b3Scalar w);
inline b3Vector4 b3MakeVector4(b3Scalar x, b3Scalar y, b3Scalar z, b3Scalar w);

/**@brief b2Vector3 can be used to represent 3D points and vectors.
 * It has an un-used w component to suit 16-byte alignment when b2Vector3 is stored in containers. This extra component can be used by derived classes (Quaternion?) or by user
 * Ideally, this class should be replaced by a platform optimized SIMD version that keeps the data in registers
 */
B2_ATTRIBUTE_ALIGNED16(class)
b2Vector3
{
public:
#if defined(B2_USE_SSE) || defined(B2_USE_NEON)  // _WIN32 || ARM
	union {
		b2SimdFloat4 mVec128;
		float m_floats[4];
		struct
		{
			float x, y, z, w;
		};
	};
#else
	union {
		float m_floats[4];
		struct
		{
			float x, y, z, w;
		};
	};
#endif

public:
	B2_DECLARE_ALIGNED_ALLOCATOR();

#if defined(B2_USE_SSE) || defined(B2_USE_NEON)  // _WIN32 || ARM
	B2_FORCE_INLINE b2SimdFloat4 get128() const
	{
		return mVec128;
	}
	B2_FORCE_INLINE void set128(b2SimdFloat4 v128)
	{
		mVec128 = v128;
	}
#endif

public:
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
    //#if defined (B2_USE_SSE) || defined (B2_USE_NEON)
    b2Vector3(b2SimdFloat4 v128)
    {
        mVec128 = v128;
    }
#endif

    b2Vector3(b2Scalar x, b2Scalar y, b2Scalar z)
	{
		m_floats[0] = x;
		m_floats[1] = y;
		m_floats[2] = z;
		m_floats[3] = b2Scalar(0.f);
	}
    
    b2Vector3(b2Scalar x, b2Scalar y, b2Scalar z, b2Scalar w);
	{
		m_floats[0] = x;
		m_floats[1] = y;
		m_floats[2] = z;
		m_floats[3] = w;
	}

	/**@brief Add a vector to this one
 * @param The vector to add to this one */
	B2_FORCE_INLINE b2Vector3& operator+=(const b2Vector3& v)
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
		mVec128 = _mm_add_ps(mVec128, v.mVec128);
#elif defined(B2_USE_NEON)
		mVec128 = vaddq_f32(mVec128, v.mVec128);
#else
		m_floats[0] += v.m_floats[0];
		m_floats[1] += v.m_floats[1];
		m_floats[2] += v.m_floats[2];
#endif
		return *this;
	}

	/**@brief Subtract a vector from this one
   * @param The vector to subtract */
	B2_FORCE_INLINE b2Vector3& operator-=(const b2Vector3& v)
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
		mVec128 = _mm_sub_ps(mVec128, v.mVec128);
#elif defined(B2_USE_NEON)
		mVec128 = vsubq_f32(mVec128, v.mVec128);
#else
		m_floats[0] -= v.m_floats[0];
		m_floats[1] -= v.m_floats[1];
		m_floats[2] -= v.m_floats[2];
#endif
		return *this;
	}

	/**@brief Scale the vector
   * @param s Scale factor */
	B2_FORCE_INLINE b2Vector3& operator*=(const b2Scalar& s)
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
		__m128 vs = _mm_load_ss(&s);  //	(S 0 0 0)
		vs = B2_pshufd_ps(vs, 0x80);  //	(S S S 0.0)
		mVec128 = _mm_mul_ps(mVec128, vs);
#elif defined(B2_USE_NEON)
		mVec128 = vmulq_n_f32(mVec128, s);
#else
		m_floats[0] *= s;
		m_floats[1] *= s;
		m_floats[2] *= s;
#endif
		return *this;
	}

	/**@brief Inversely scale the vector
   * @param s Scale factor to divide by */
	B2_FORCE_INLINE b2Vector3& operator/=(const b2Scalar& s)
	{
		b2FullAssert(s != b2Scalar(0.0));

#if 0  //defined(B2_USE_SSE_IN_API)
// this code is not faster !
		__m128 vs = _mm_load_ss(&s);
		vs = _mm_div_ss(b2v1110, vs);
		vs = B2_pshufd_ps(vs, 0x00);	//	(S S S S)

		mVec128 = _mm_mul_ps(mVec128, vs);

		return *this;
#else
		return *this *= b2Scalar(1.0) / s;
#endif
	}

	/**@brief Return the dot product
   * @param v The other vector in the dot product */
	B2_FORCE_INLINE b2Scalar dot(const b2Vector3& v) const
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
		__m128 vd = _mm_mul_ps(mVec128, v.mVec128);
		__m128 z = _mm_movehl_ps(vd, vd);
		__m128 y = _mm_shuffle_ps(vd, vd, 0x55);
		vd = _mm_add_ss(vd, y);
		vd = _mm_add_ss(vd, z);
		return _mm_cvtss_f32(vd);
#elif defined(B2_USE_NEON)
		float32x4_t vd = vmulq_f32(mVec128, v.mVec128);
		float32x2_t x = vpadd_f32(vget_low_f32(vd), vget_low_f32(vd));
		x = vadd_f32(x, vget_high_f32(vd));
		return vget_lane_f32(x, 0);
#else
		return m_floats[0] * v.m_floats[0] +
			   m_floats[1] * v.m_floats[1] +
			   m_floats[2] * v.m_floats[2];
#endif
	}

	/**@brief Return the length of the vector squared */
	B2_FORCE_INLINE b2Scalar length2() const
	{
		return dot(*this);
	}

	/**@brief Return the length of the vector */
	B2_FORCE_INLINE b2Scalar length() const
	{
		return b2Sqrt(length2());
	}

	/**@brief Return the distance squared between the ends of this and another vector
   * This is symantically treating the vector like a point */
	B2_FORCE_INLINE b2Scalar distance2(const b2Vector3& v) const;

	/**@brief Return the distance between the ends of this and another vector
   * This is symantically treating the vector like a point */
	B2_FORCE_INLINE b2Scalar distance(const b2Vector3& v) const;

	B2_FORCE_INLINE b2Vector3& safeNormalize()
	{
		b2Scalar l2 = length2();
		//triNormal.normalize();
		if (l2 >= B2_EPSILON * B2_EPSILON)
		{
			(*this) /= b2Sqrt(l2);
		}
		else
		{
			setValue(1, 0, 0);
		}
		return *this;
	}

	/**@brief Normalize this vector
   * x^2 + y^2 + z^2 = 1 */
	B2_FORCE_INLINE b2Vector3& normalize()
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
		// dot product first
		__m128 vd = _mm_mul_ps(mVec128, mVec128);
		__m128 z = _mm_movehl_ps(vd, vd);
		__m128 y = _mm_shuffle_ps(vd, vd, 0x55);
		vd = _mm_add_ss(vd, y);
		vd = _mm_add_ss(vd, z);

#if 0
        vd = _mm_sqrt_ss(vd);
		vd = _mm_div_ss(b2v1110, vd);
		vd = B2_splat_ps(vd, 0x80);
		mVec128 = _mm_mul_ps(mVec128, vd);
#else

		// NR step 1/sqrt(x) - vd is x, y is output
		y = _mm_rsqrt_ss(vd);  // estimate

		//  one step NR
		z = b2v1_5;
		vd = _mm_mul_ss(vd, b2vHalf);  // vd * 0.5
		//x2 = vd;
		vd = _mm_mul_ss(vd, y);  // vd * 0.5 * y0
		vd = _mm_mul_ss(vd, y);  // vd * 0.5 * y0 * y0
		z = _mm_sub_ss(z, vd);   // 1.5 - vd * 0.5 * y0 * y0

		y = _mm_mul_ss(y, z);  // y0 * (1.5 - vd * 0.5 * y0 * y0)

		y = B2_splat_ps(y, 0x80);
		mVec128 = _mm_mul_ps(mVec128, y);

#endif

		return *this;
#else
		return *this /= length();
#endif
	}

	/**@brief Return a normalized version of this vector */
	B2_FORCE_INLINE b2Vector3 normalized() const;

	/**@brief Return a rotated version of this vector
   * @param wAxis The axis to rotate about
   * @param angle The angle to rotate by */
	B2_FORCE_INLINE b2Vector3 rotate(const b2Vector3& wAxis, const b2Scalar angle) const;

	/**@brief Return the angle between this and another vector
   * @param v The other vector */
	B2_FORCE_INLINE b2Scalar angle(const b2Vector3& v) const
	{
		b2Scalar s = b2Sqrt(length2() * v.length2());
		b2FullAssert(s != b2Scalar(0.0));
		return b2Acos(dot(v) / s);
	}

	/**@brief Return a vector will the absolute values of each element */
	B2_FORCE_INLINE b2Vector3 absolute() const
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
		return b2MakeVector3(_mm_and_ps(mVec128, b2v3AbsfMask));
#elif defined(B2_USE_NEON)
		return b2Vector3(vabsq_f32(mVec128));
#else
		return b2MakeVector3(
			b2Fabs(m_floats[0]),
			b2Fabs(m_floats[1]),
			b2Fabs(m_floats[2]));
#endif
	}

	/**@brief Return the cross product between this and another vector
   * @param v The other vector */
	B2_FORCE_INLINE b2Vector3 cross(const b2Vector3& v) const
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
		__m128 T, V;

		T = B2_pshufd_ps(mVec128, B2_SHUFFLE(1, 2, 0, 3));    //	(Y Z X 0)
		V = B2_pshufd_ps(v.mVec128, B2_SHUFFLE(1, 2, 0, 3));  //	(Y Z X 0)

		V = _mm_mul_ps(V, mVec128);
		T = _mm_mul_ps(T, v.mVec128);
		V = _mm_sub_ps(V, T);

		V = B2_pshufd_ps(V, B2_SHUFFLE(1, 2, 0, 3));
		return b2MakeVector3(V);
#elif defined(B2_USE_NEON)
		float32x4_t T, V;
		// form (Y, Z, X, _) of mVec128 and v.mVec128
		float32x2_t Tlow = vget_low_f32(mVec128);
		float32x2_t Vlow = vget_low_f32(v.mVec128);
		T = vcombine_f32(vext_f32(Tlow, vget_high_f32(mVec128), 1), Tlow);
		V = vcombine_f32(vext_f32(Vlow, vget_high_f32(v.mVec128), 1), Vlow);

		V = vmulq_f32(V, mVec128);
		T = vmulq_f32(T, v.mVec128);
		V = vsubq_f32(V, T);
		Vlow = vget_low_f32(V);
		// form (Y, Z, X, _);
		V = vcombine_f32(vext_f32(Vlow, vget_high_f32(V), 1), Vlow);
		V = (float32x4_t)vandq_s32((int32x4_t)V, b2vFFF0Mask);

		return b2Vector3(V);
#else
		return b2MakeVector3(
			m_floats[1] * v.m_floats[2] - m_floats[2] * v.m_floats[1],
			m_floats[2] * v.m_floats[0] - m_floats[0] * v.m_floats[2],
			m_floats[0] * v.m_floats[1] - m_floats[1] * v.m_floats[0]);
#endif
	}

	B2_FORCE_INLINE b2Scalar triple(const b2Vector3& v1, const b2Vector3& v2) const
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
		// cross:
		__m128 T = _mm_shuffle_ps(v1.mVec128, v1.mVec128, B2_SHUFFLE(1, 2, 0, 3));  //	(Y Z X 0)
		__m128 V = _mm_shuffle_ps(v2.mVec128, v2.mVec128, B2_SHUFFLE(1, 2, 0, 3));  //	(Y Z X 0)

		V = _mm_mul_ps(V, v1.mVec128);
		T = _mm_mul_ps(T, v2.mVec128);
		V = _mm_sub_ps(V, T);

		V = _mm_shuffle_ps(V, V, B2_SHUFFLE(1, 2, 0, 3));

		// dot:
		V = _mm_mul_ps(V, mVec128);
		__m128 z = _mm_movehl_ps(V, V);
		__m128 y = _mm_shuffle_ps(V, V, 0x55);
		V = _mm_add_ss(V, y);
		V = _mm_add_ss(V, z);
		return _mm_cvtss_f32(V);

#elif defined(B2_USE_NEON)
		// cross:
		float32x4_t T, V;
		// form (Y, Z, X, _) of mVec128 and v.mVec128
		float32x2_t Tlow = vget_low_f32(v1.mVec128);
		float32x2_t Vlow = vget_low_f32(v2.mVec128);
		T = vcombine_f32(vext_f32(Tlow, vget_high_f32(v1.mVec128), 1), Tlow);
		V = vcombine_f32(vext_f32(Vlow, vget_high_f32(v2.mVec128), 1), Vlow);

		V = vmulq_f32(V, v1.mVec128);
		T = vmulq_f32(T, v2.mVec128);
		V = vsubq_f32(V, T);
		Vlow = vget_low_f32(V);
		// form (Y, Z, X, _);
		V = vcombine_f32(vext_f32(Vlow, vget_high_f32(V), 1), Vlow);

		// dot:
		V = vmulq_f32(mVec128, V);
		float32x2_t x = vpadd_f32(vget_low_f32(V), vget_low_f32(V));
		x = vadd_f32(x, vget_high_f32(V));
		return vget_lane_f32(x, 0);
#else
		return m_floats[0] * (v1.m_floats[1] * v2.m_floats[2] - v1.m_floats[2] * v2.m_floats[1]) +
			   m_floats[1] * (v1.m_floats[2] * v2.m_floats[0] - v1.m_floats[0] * v2.m_floats[2]) +
			   m_floats[2] * (v1.m_floats[0] * v2.m_floats[1] - v1.m_floats[1] * v2.m_floats[0]);
#endif
	}

	/**@brief Return the axis with the smallest value
   * Note return values are 0,1,2 for x, y, or z */
	B2_FORCE_INLINE int minAxis() const
	{
		return m_floats[0] < m_floats[1] ? (m_floats[0] < m_floats[2] ? 0 : 2) : (m_floats[1] < m_floats[2] ? 1 : 2);
	}

	/**@brief Return the axis with the largest value
   * Note return values are 0,1,2 for x, y, or z */
	B2_FORCE_INLINE int maxAxis() const
	{
		return m_floats[0] < m_floats[1] ? (m_floats[1] < m_floats[2] ? 2 : 1) : (m_floats[0] < m_floats[2] ? 2 : 0);
	}

	B2_FORCE_INLINE int furthestAxis() const
	{
		return absolute().minAxis();
	}

	B2_FORCE_INLINE int closestAxis() const
	{
		return absolute().maxAxis();
	}

	B2_FORCE_INLINE void setInterpolate3(const b2Vector3& v0, const b2Vector3& v1, b2Scalar rt)
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
		__m128 vrt = _mm_load_ss(&rt);  //	(rt 0 0 0)
		b2Scalar s = b2Scalar(1.0) - rt;
		__m128 vs = _mm_load_ss(&s);  //	(S 0 0 0)
		vs = B2_pshufd_ps(vs, 0x80);  //	(S S S 0.0)
		__m128 r0 = _mm_mul_ps(v0.mVec128, vs);
		vrt = B2_pshufd_ps(vrt, 0x80);  //	(rt rt rt 0.0)
		__m128 r1 = _mm_mul_ps(v1.mVec128, vrt);
		__m128 tmp3 = _mm_add_ps(r0, r1);
		mVec128 = tmp3;
#elif defined(B2_USE_NEON)
		float32x4_t vl = vsubq_f32(v1.mVec128, v0.mVec128);
		vl = vmulq_n_f32(vl, rt);
		mVec128 = vaddq_f32(vl, v0.mVec128);
#else
		b2Scalar s = b2Scalar(1.0) - rt;
		m_floats[0] = s * v0.m_floats[0] + rt * v1.m_floats[0];
		m_floats[1] = s * v0.m_floats[1] + rt * v1.m_floats[1];
		m_floats[2] = s * v0.m_floats[2] + rt * v1.m_floats[2];
		//don't do the unused w component
		//		m_co[3] = s * v0[3] + rt * v1[3];
#endif
	}

	/**@brief Return the linear interpolation between this and another vector
   * @param v The other vector
   * @param t The ration of this to v (t = 0 => return this, t=1 => return other) */
	B2_FORCE_INLINE b2Vector3 lerp(const b2Vector3& v, const b2Scalar& t) const
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
		__m128 vt = _mm_load_ss(&t);  //	(t 0 0 0)
		vt = B2_pshufd_ps(vt, 0x80);  //	(rt rt rt 0.0)
		__m128 vl = _mm_sub_ps(v.mVec128, mVec128);
		vl = _mm_mul_ps(vl, vt);
		vl = _mm_add_ps(vl, mVec128);

		return b2MakeVector3(vl);
#elif defined(B2_USE_NEON)
		float32x4_t vl = vsubq_f32(v.mVec128, mVec128);
		vl = vmulq_n_f32(vl, t);
		vl = vaddq_f32(vl, mVec128);

		return b2Vector3(vl);
#else
		return b2MakeVector3(m_floats[0] + (v.m_floats[0] - m_floats[0]) * t,
							 m_floats[1] + (v.m_floats[1] - m_floats[1]) * t,
							 m_floats[2] + (v.m_floats[2] - m_floats[2]) * t);
#endif
	}

	/**@brief Elementwise multiply this vector by the other
   * @param v The other vector */
	B2_FORCE_INLINE b2Vector3& operator*=(const b2Vector3& v)
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
		mVec128 = _mm_mul_ps(mVec128, v.mVec128);
#elif defined(B2_USE_NEON)
		mVec128 = vmulq_f32(mVec128, v.mVec128);
#else
		m_floats[0] *= v.m_floats[0];
		m_floats[1] *= v.m_floats[1];
		m_floats[2] *= v.m_floats[2];
#endif
		return *this;
	}

	/**@brief Return the x value */
	B2_FORCE_INLINE const b2Scalar& getX() const { return m_floats[0]; }
	/**@brief Return the y value */
	B2_FORCE_INLINE const b2Scalar& getY() const { return m_floats[1]; }
	/**@brief Return the z value */
	B2_FORCE_INLINE const b2Scalar& getZ() const { return m_floats[2]; }
	/**@brief Return the w value */
	B2_FORCE_INLINE const b2Scalar& getW() const { return m_floats[3]; }

	/**@brief Set the x value */
	B2_FORCE_INLINE void setX(b2Scalar _x) { m_floats[0] = _x; };
	/**@brief Set the y value */
	B2_FORCE_INLINE void setY(b2Scalar _y) { m_floats[1] = _y; };
	/**@brief Set the z value */
	B2_FORCE_INLINE void setZ(b2Scalar _z) { m_floats[2] = _z; };
	/**@brief Set the w value */
	B2_FORCE_INLINE void setW(b2Scalar _w) { m_floats[3] = _w; };

	//B2_FORCE_INLINE b2Scalar&       operator[](int i)       { return (&m_floats[0])[i];	}
	//B2_FORCE_INLINE const b2Scalar& operator[](int i) const { return (&m_floats[0])[i]; }
	///operator b2Scalar*() replaces operator[], using implicit conversion. We added operator != and operator == to avoid pointer comparisons.
	B2_FORCE_INLINE operator b2Scalar*() { return &m_floats[0]; }
	B2_FORCE_INLINE operator const b2Scalar*() const { return &m_floats[0]; }

	B2_FORCE_INLINE bool operator==(const b2Vector3& other) const
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
		return (0xf == _mm_movemask_ps((__m128)_mm_cmpeq_ps(mVec128, other.mVec128)));
#else
		return ((m_floats[3] == other.m_floats[3]) &&
				(m_floats[2] == other.m_floats[2]) &&
				(m_floats[1] == other.m_floats[1]) &&
				(m_floats[0] == other.m_floats[0]));
#endif
	}

	B2_FORCE_INLINE bool operator!=(const b2Vector3& other) const
	{
		return !(*this == other);
	}

	/**@brief Set each element to the max of the current values and the values of another b2Vector3
   * @param other The other b2Vector3 to compare with
   */
	B2_FORCE_INLINE void setMax(const b2Vector3& other)
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
		mVec128 = _mm_max_ps(mVec128, other.mVec128);
#elif defined(B2_USE_NEON)
		mVec128 = vmaxq_f32(mVec128, other.mVec128);
#else
		b2SetMax(m_floats[0], other.m_floats[0]);
		b2SetMax(m_floats[1], other.m_floats[1]);
		b2SetMax(m_floats[2], other.m_floats[2]);
		b2SetMax(m_floats[3], other.m_floats[3]);
#endif
	}

	/**@brief Set each element to the min of the current values and the values of another b2Vector3
   * @param other The other b2Vector3 to compare with
   */
	B2_FORCE_INLINE void setMin(const b2Vector3& other)
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
		mVec128 = _mm_min_ps(mVec128, other.mVec128);
#elif defined(B2_USE_NEON)
		mVec128 = vminq_f32(mVec128, other.mVec128);
#else
		b2SetMin(m_floats[0], other.m_floats[0]);
		b2SetMin(m_floats[1], other.m_floats[1]);
		b2SetMin(m_floats[2], other.m_floats[2]);
		b2SetMin(m_floats[3], other.m_floats[3]);
#endif
	}

	B2_FORCE_INLINE void setZero()
	{
		m_floats[0] = b2Scalar(0.f);
		m_floats[1] = b2Scalar(0.f);
		m_floats[2] = b2Scalar(0.f);
		m_floats[3] = b2Scalar(0.f);
	}

	B2_FORCE_INLINE void setValue(const b2Scalar& _x, const b2Scalar& _y, const b2Scalar& _z)
	{
		m_floats[0] = _x;
		m_floats[1] = _y;
		m_floats[2] = _z;
		m_floats[3] = b2Scalar(0.f);
	}

	void getSkewSymmetricMatrix(b2Vector3 * v0, b2Vector3 * v1, b2Vector3 * v2) const
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)

		__m128 V = _mm_and_ps(mVec128, b2vFFF0fMask);
		__m128 V0 = _mm_xor_ps(b2vMzeroMask, V);
		__m128 V2 = _mm_movelh_ps(V0, V);

		__m128 V1 = _mm_shuffle_ps(V, V0, 0xCE);

		V0 = _mm_shuffle_ps(V0, V, 0xDB);
		V2 = _mm_shuffle_ps(V2, V, 0xF9);

		v0->mVec128 = V0;
		v1->mVec128 = V1;
		v2->mVec128 = V2;
#else
		v0->setValue(0., -getZ(), getY());
		v1->setValue(getZ(), 0., -getX());
		v2->setValue(-getY(), getX(), 0.);
#endif
	}

	void setZero()
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
		mVec128 = (__m128)_mm_xor_ps(mVec128, mVec128);
#elif defined(B2_USE_NEON)
		int32x4_t vi = vdupq_n_s32(0);
		mVec128 = vreinterpretq_f32_s32(vi);
#else
		setValue(b2Scalar(0.), b2Scalar(0.), b2Scalar(0.));
#endif
	}

	B2_FORCE_INLINE bool isZero() const
	{
		return m_floats[0] == b2Scalar(0) && m_floats[1] == b2Scalar(0) && m_floats[2] == b2Scalar(0);
	}

	B2_FORCE_INLINE bool fuzzyZero() const
	{
		return length2() < B2_EPSILON;
	}

	B2_FORCE_INLINE void serialize(struct b2Vector3Data & dataOut) const;

	B2_FORCE_INLINE void deSerialize(const struct b2Vector3Data& dataIn);

	B2_FORCE_INLINE void serializeFloat(struct b2Vector3FloatData & dataOut) const;

	B2_FORCE_INLINE void deSerializeFloat(const struct b2Vector3FloatData& dataIn);

	B2_FORCE_INLINE void serializeDouble(struct b2Vector3DoubleData & dataOut) const;

	B2_FORCE_INLINE void deSerializeDouble(const struct b2Vector3DoubleData& dataIn);

	/**@brief returns index of maximum dot product between this and vectors in array[]
         * @param array The other vectors
         * @param array_count The number of other vectors
         * @param dotOut The maximum dot product */
	B2_FORCE_INLINE long maxDot(const b2Vector3* array, long array_count, b2Scalar& dotOut) const;

	/**@brief returns index of minimum dot product between this and vectors in array[]
         * @param array The other vectors
         * @param array_count The number of other vectors
         * @param dotOut The minimum dot product */
	B2_FORCE_INLINE long minDot(const b2Vector3* array, long array_count, b2Scalar& dotOut) const;

	/* create a vector as  b2Vector3( this->dot( b2Vector3 v0 ), this->dot( b2Vector3 v1), this->dot( b2Vector3 v2 ))  */
	B2_FORCE_INLINE b2Vector3 dot3(const b2Vector3& v0, const b2Vector3& v1, const b2Vector3& v2) const
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)

		__m128 a0 = _mm_mul_ps(v0.mVec128, this->mVec128);
		__m128 a1 = _mm_mul_ps(v1.mVec128, this->mVec128);
		__m128 a2 = _mm_mul_ps(v2.mVec128, this->mVec128);
		__m128 b0 = _mm_unpacklo_ps(a0, a1);
		__m128 b1 = _mm_unpackhi_ps(a0, a1);
		__m128 b2 = _mm_unpacklo_ps(a2, _mm_setzero_ps());
		__m128 r = _mm_movelh_ps(b0, b2);
		r = _mm_add_ps(r, _mm_movehl_ps(b2, b0));
		a2 = _mm_and_ps(a2, b2vxyzMaskf);
		r = _mm_add_ps(r, b2CastdTo128f(_mm_move_sd(b2CastfTo128d(a2), b2CastfTo128d(b1))));
		return b2MakeVector3(r);

#elif defined(B2_USE_NEON)
		static const uint32x4_t xyzMask = (const uint32x4_t){-1, -1, -1, 0};
		float32x4_t a0 = vmulq_f32(v0.mVec128, this->mVec128);
		float32x4_t a1 = vmulq_f32(v1.mVec128, this->mVec128);
		float32x4_t a2 = vmulq_f32(v2.mVec128, this->mVec128);
		float32x2x2_t zLo = vtrn_f32(vget_high_f32(a0), vget_high_f32(a1));
		a2 = (float32x4_t)vandq_u32((uint32x4_t)a2, xyzMask);
		float32x2_t b0 = vadd_f32(vpadd_f32(vget_low_f32(a0), vget_low_f32(a1)), zLo.val[0]);
		float32x2_t b1 = vpadd_f32(vpadd_f32(vget_low_f32(a2), vget_high_f32(a2)), vdup_n_f32(0.0f));
		return b2Vector3(vcombine_f32(b0, b1));
#else
		return b2MakeVector3(dot(v0), dot(v1), dot(v2));
#endif
	}
};

/**@brief Return the sum of two vectors (Point symantics)*/
B2_FORCE_INLINE b2Vector3
operator+(const b2Vector3& v1, const b2Vector3& v2)
{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
	return b2MakeVector3(_mm_add_ps(v1.mVec128, v2.mVec128));
#elif defined(B2_USE_NEON)
	return b2MakeVector3(vaddq_f32(v1.mVec128, v2.mVec128));
#else
	return b2MakeVector3(
		v1.m_floats[0] + v2.m_floats[0],
		v1.m_floats[1] + v2.m_floats[1],
		v1.m_floats[2] + v2.m_floats[2]);
#endif
}

/**@brief Return the elementwise product of two vectors */
B2_FORCE_INLINE b2Vector3
operator*(const b2Vector3& v1, const b2Vector3& v2)
{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
	return b2MakeVector3(_mm_mul_ps(v1.mVec128, v2.mVec128));
#elif defined(B2_USE_NEON)
	return b2MakeVector3(vmulq_f32(v1.mVec128, v2.mVec128));
#else
	return b2MakeVector3(
		v1.m_floats[0] * v2.m_floats[0],
		v1.m_floats[1] * v2.m_floats[1],
		v1.m_floats[2] * v2.m_floats[2]);
#endif
}

/**@brief Return the difference between two vectors */
B2_FORCE_INLINE b2Vector3
operator-(const b2Vector3& v1, const b2Vector3& v2)
{
#if (defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE))

	//	without _mm_and_ps this code causes slowdown in Concave moving
	__m128 r = _mm_sub_ps(v1.mVec128, v2.mVec128);
	return b2MakeVector3(_mm_and_ps(r, b2vFFF0fMask));
#elif defined(B2_USE_NEON)
	float32x4_t r = vsubq_f32(v1.mVec128, v2.mVec128);
	return b2MakeVector3((float32x4_t)vandq_s32((int32x4_t)r, b2vFFF0Mask));
#else
	return b2MakeVector3(
		v1.m_floats[0] - v2.m_floats[0],
		v1.m_floats[1] - v2.m_floats[1],
		v1.m_floats[2] - v2.m_floats[2]);
#endif
}

/**@brief Return the negative of the vector */
B2_FORCE_INLINE b2Vector3
operator-(const b2Vector3& v)
{
#if (defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE))
	__m128 r = _mm_xor_ps(v.mVec128, b2vMzeroMask);
	return b2MakeVector3(_mm_and_ps(r, b2vFFF0fMask));
#elif defined(B2_USE_NEON)
	return b2MakeVector3((b2SimdFloat4)veorq_s32((int32x4_t)v.mVec128, (int32x4_t)b2vMzeroMask));
#else
	return b2MakeVector3(-v.m_floats[0], -v.m_floats[1], -v.m_floats[2]);
#endif
}

/**@brief Return the vector scaled by s */
B2_FORCE_INLINE b2Vector3
operator*(const b2Vector3& v, const b2Scalar& s)
{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
	__m128 vs = _mm_load_ss(&s);  //	(S 0 0 0)
	vs = B2_pshufd_ps(vs, 0x80);  //	(S S S 0.0)
	return b2MakeVector3(_mm_mul_ps(v.mVec128, vs));
#elif defined(B2_USE_NEON)
	float32x4_t r = vmulq_n_f32(v.mVec128, s);
	return b2MakeVector3((float32x4_t)vandq_s32((int32x4_t)r, b2vFFF0Mask));
#else
	return b2MakeVector3(v.m_floats[0] * s, v.m_floats[1] * s, v.m_floats[2] * s);
#endif
}

/**@brief Return the vector scaled by s */
B2_FORCE_INLINE b2Vector3
operator*(const b2Scalar& s, const b2Vector3& v)
{
	return v * s;
}

/**@brief Return the vector inversely scaled by s */
B2_FORCE_INLINE b2Vector3
operator/(const b2Vector3& v, const b2Scalar& s)
{
	b2FullAssert(s != b2Scalar(0.0));
#if 0  //defined(B2_USE_SSE_IN_API)
// this code is not faster !
	__m128 vs = _mm_load_ss(&s);
    vs = _mm_div_ss(b2v1110, vs);
	vs = B2_pshufd_ps(vs, 0x00);	//	(S S S S)

	return b2Vector3(_mm_mul_ps(v.mVec128, vs));
#else
	return v * (b2Scalar(1.0) / s);
#endif
}

/**@brief Return the vector inversely scaled by s */
B2_FORCE_INLINE b2Vector3
operator/(const b2Vector3& v1, const b2Vector3& v2)
{
#if (defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE))
	__m128 vec = _mm_div_ps(v1.mVec128, v2.mVec128);
	vec = _mm_and_ps(vec, b2vFFF0fMask);
	return b2MakeVector3(vec);
#elif defined(B2_USE_NEON)
	float32x4_t x, y, v, m;

	x = v1.mVec128;
	y = v2.mVec128;

	v = vrecpeq_f32(y);     // v ~ 1/y
	m = vrecpsq_f32(y, v);  // m = (2-v*y)
	v = vmulq_f32(v, m);    // vv = v*m ~~ 1/y
	m = vrecpsq_f32(y, v);  // mm = (2-vv*y)
	v = vmulq_f32(v, x);    // x*vv
	v = vmulq_f32(v, m);    // (x*vv)*(2-vv*y) = x*(vv(2-vv*y)) ~~~ x/y

	return b2Vector3(v);
#else
	return b2MakeVector3(
		v1.m_floats[0] / v2.m_floats[0],
		v1.m_floats[1] / v2.m_floats[1],
		v1.m_floats[2] / v2.m_floats[2]);
#endif
}

/**@brief Return the dot product between two vectors */
B2_FORCE_INLINE b2Scalar
b2Dot(const b2Vector3& v1, const b2Vector3& v2)
{
	return v1.dot(v2);
}

/**@brief Return the distance squared between two vectors */
B2_FORCE_INLINE b2Scalar
b2Distance2(const b2Vector3& v1, const b2Vector3& v2)
{
	return v1.distance2(v2);
}

/**@brief Return the distance between two vectors */
B2_FORCE_INLINE b2Scalar
b2Distance(const b2Vector3& v1, const b2Vector3& v2)
{
	return v1.distance(v2);
}

/**@brief Return the angle between two vectors */
B2_FORCE_INLINE b2Scalar
b2Angle(const b2Vector3& v1, const b2Vector3& v2)
{
	return v1.angle(v2);
}

/**@brief Return the cross product of two vectors */
B2_FORCE_INLINE b2Vector3
b2Cross(const b2Vector3& v1, const b2Vector3& v2)
{
	return v1.cross(v2);
}

B2_FORCE_INLINE b2Scalar
b2Triple(const b2Vector3& v1, const b2Vector3& v2, const b2Vector3& v3)
{
	return v1.triple(v2, v3);
}

/**@brief Return the linear interpolation between two vectors
 * @param v1 One vector
 * @param v2 The other vector
 * @param t The ration of this to v (t = 0 => return v1, t=1 => return v2) */
B2_FORCE_INLINE b2Vector3
b2Lerp(const b2Vector3& v1, const b2Vector3& v2, const b2Scalar& t)
{
	return v1.lerp(v2, t);
}

B2_FORCE_INLINE b2Scalar b2Vector3::distance2(const b2Vector3& v) const
{
	return (v - *this).length2();
}

B2_FORCE_INLINE b2Scalar b2Vector3::distance(const b2Vector3& v) const
{
	return (v - *this).length();
}

B2_FORCE_INLINE b2Vector3 b2Vector3::normalized() const
{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
	b2Vector3 norm = *this;

	return norm.normalize();
#else
	return *this / length();
#endif
}

B2_FORCE_INLINE b2Vector3 b2Vector3::rotate(const b2Vector3& wAxis, const b2Scalar _angle) const
{
	// wAxis must be a unit lenght vector

#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)

	__m128 O = _mm_mul_ps(wAxis.mVec128, mVec128);
	b2Scalar ssin = b2Sin(_angle);
	__m128 C = wAxis.cross(b2MakeVector3(mVec128)).mVec128;
	O = _mm_and_ps(O, b2vFFF0fMask);
	b2Scalar scos = b2Cos(_angle);

	__m128 vsin = _mm_load_ss(&ssin);  //	(S 0 0 0)
	__m128 vcos = _mm_load_ss(&scos);  //	(S 0 0 0)

	__m128 Y = B2_pshufd_ps(O, 0xC9);  //	(Y Z X 0)
	__m128 Z = B2_pshufd_ps(O, 0xD2);  //	(Z X Y 0)
	O = _mm_add_ps(O, Y);
	vsin = B2_pshufd_ps(vsin, 0x80);  //	(S S S 0)
	O = _mm_add_ps(O, Z);
	vcos = B2_pshufd_ps(vcos, 0x80);  //	(S S S 0)

	vsin = vsin * C;
	O = O * wAxis.mVec128;
	__m128 X = mVec128 - O;

	O = O + vsin;
	vcos = vcos * X;
	O = O + vcos;

	return b2MakeVector3(O);
#else
	b2Vector3 o = wAxis * wAxis.dot(*this);
	b2Vector3 _x = *this - o;
	b2Vector3 _y;

	_y = wAxis.cross(*this);

	return (o + _x * b2Cos(_angle) + _y * b2Sin(_angle));
#endif
}

B2_FORCE_INLINE long b2Vector3::maxDot(const b2Vector3* array, long array_count, b2Scalar& dotOut) const
{
#if defined(B2_USE_SSE) || defined(B2_USE_NEON)
#if defined _WIN32 || defined(B2_USE_SSE)
	const long scalar_cutoff = 10;
	long B2_maxdot_large(const float* array, const float* vec, unsigned long array_count, float* dotOut);
#elif defined B2_USE_NEON
	const long scalar_cutoff = 4;
	extern long (*_maxdot_large)(const float* array, const float* vec, unsigned long array_count, float* dotOut);
#endif
	if (array_count < scalar_cutoff)
#else

#endif  //B2_USE_SSE || B2_USE_NEON
	{
		b2Scalar maxDot = -B2_INFINITY;
		int i = 0;
		int ptIndex = -1;
		for (i = 0; i < array_count; i++)
		{
			b2Scalar dot = array[i].dot(*this);

			if (dot > maxDot)
			{
				maxDot = dot;
				ptIndex = i;
			}
		}

		b2Assert(ptIndex >= 0);
		if (ptIndex < 0)
		{
			ptIndex = 0;
		}
		dotOut = maxDot;
		return ptIndex;
	}
#if defined(B2_USE_SSE) || defined(B2_USE_NEON)
	return B2_maxdot_large((float*)array, (float*)&m_floats[0], array_count, &dotOut);
#endif
}

B2_FORCE_INLINE long b2Vector3::minDot(const b2Vector3* array, long array_count, b2Scalar& dotOut) const
{
#if defined(B2_USE_SSE) || defined(B2_USE_NEON)
#if defined B2_USE_SSE
	const long scalar_cutoff = 10;
	long B2_mindot_large(const float* array, const float* vec, unsigned long array_count, float* dotOut);
#elif defined B2_USE_NEON
	const long scalar_cutoff = 4;
	extern long (*B2_mindot_large)(const float* array, const float* vec, unsigned long array_count, float* dotOut);
#else
#error unhandled arch!
#endif

	if (array_count < scalar_cutoff)
#endif  //B2_USE_SSE || B2_USE_NEON
	{
		b2Scalar minDot = B2_INFINITY;
		int i = 0;
		int ptIndex = -1;

		for (i = 0; i < array_count; i++)
		{
			b2Scalar dot = array[i].dot(*this);

			if (dot < minDot)
			{
				minDot = dot;
				ptIndex = i;
			}
		}

		dotOut = minDot;

		return ptIndex;
	}
#if defined(B2_USE_SSE) || defined(B2_USE_NEON)
	return B2_mindot_large((float*)array, (float*)&m_floats[0], array_count, &dotOut);
#endif
}

class b2Vector4 : public b2Vector3
{
public:
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
    b2Vector4(b2SimdFloat4 vec);
#endif

    b2Vector4(b2Scalar x, b2Scalar y, b2Scalar z, b2Scalar w);

	B2_FORCE_INLINE b2Vector4 absolute4() const
	{
#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)
		return b2MakeVector4(_mm_and_ps(mVec128, b2vAbsfMask));
#elif defined(B2_USE_NEON)
		return b2Vector4(vabsq_f32(mVec128));
#else
		return b2MakeVector4(
			b2Fabs(m_floats[0]),
			b2Fabs(m_floats[1]),
			b2Fabs(m_floats[2]),
			b2Fabs(m_floats[3]));
#endif
	}

	b2Scalar getW() const { return m_floats[3]; }

	B2_FORCE_INLINE int maxAxis4() const
	{
		int maxIndex = -1;
		b2Scalar maxVal = b2Scalar(-B2_LARGE_FLOAT);
		if (m_floats[0] > maxVal)
		{
			maxIndex = 0;
			maxVal = m_floats[0];
		}
		if (m_floats[1] > maxVal)
		{
			maxIndex = 1;
			maxVal = m_floats[1];
		}
		if (m_floats[2] > maxVal)
		{
			maxIndex = 2;
			maxVal = m_floats[2];
		}
		if (m_floats[3] > maxVal)
		{
			maxIndex = 3;
		}

		return maxIndex;
	}

	B2_FORCE_INLINE int minAxis4() const
	{
		int minIndex = -1;
		b2Scalar minVal = b2Scalar(B2_LARGE_FLOAT);
		if (m_floats[0] < minVal)
		{
			minIndex = 0;
			minVal = m_floats[0];
		}
		if (m_floats[1] < minVal)
		{
			minIndex = 1;
			minVal = m_floats[1];
		}
		if (m_floats[2] < minVal)
		{
			minIndex = 2;
			minVal = m_floats[2];
		}
		if (m_floats[3] < minVal)
		{
			minIndex = 3;
			minVal = m_floats[3];
		}

		return minIndex;
	}

	B2_FORCE_INLINE int closestAxis4() const
	{
		return absolute4().maxAxis4();
	}

	/**@brief Set x,y,z and zero w
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   */

	/*		void getValue(b2Scalar *m) const
		{
			m[0] = m_floats[0];
			m[1] = m_floats[1];
			m[2] =m_floats[2];
		}
*/
	/**@brief Set the values
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   * @param w Value of w
   */
	B2_FORCE_INLINE void setValue(const b2Scalar& _x, const b2Scalar& _y, const b2Scalar& _z, const b2Scalar& _w)
	{
		m_floats[0] = _x;
		m_floats[1] = _y;
		m_floats[2] = _z;
		m_floats[3] = _w;
	}
};

///b2SwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
B2_FORCE_INLINE void b2SwapScalarEndian(const b2Scalar& sourceVal, b2Scalar& destVal)
{
#ifdef B2_USE_DOUBLE_PRECISION
	unsigned char* dest = (unsigned char*)&destVal;
	unsigned char* src = (unsigned char*)&sourceVal;
	dest[0] = src[7];
	dest[1] = src[6];
	dest[2] = src[5];
	dest[3] = src[4];
	dest[4] = src[3];
	dest[5] = src[2];
	dest[6] = src[1];
	dest[7] = src[0];
#else
	unsigned char* dest = (unsigned char*)&destVal;
	unsigned char* src = (unsigned char*)&sourceVal;
	dest[0] = src[3];
	dest[1] = src[2];
	dest[2] = src[1];
	dest[3] = src[0];
#endif  //B2_USE_DOUBLE_PRECISION
}
///b2SwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
B2_FORCE_INLINE void b2SwapVector3Endian(const b2Vector3& sourceVec, b2Vector3& destVec)
{
	for (int i = 0; i < 4; i++)
	{
		b2SwapScalarEndian(sourceVec[i], destVec[i]);
	}
}

///b2UnSwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
B2_FORCE_INLINE void b2UnSwapVector3Endian(b2Vector3& vector)
{
	b2Vector3 swappedVec;
	for (int i = 0; i < 4; i++)
	{
		b2SwapScalarEndian(vector[i], swappedVec[i]);
	}
	vector = swappedVec;
}

template <class T>
B2_FORCE_INLINE void b2PlaneSpace1(const T& n, T& p, T& q)
{
	if (b2Fabs(n[2]) > B2_SQRT12)
	{
		// choose p in y-z plane
		b2Scalar a = n[1] * n[1] + n[2] * n[2];
		b2Scalar k = b2RecipSqrt(a);
		p[0] = 0;
		p[1] = -n[2] * k;
		p[2] = n[1] * k;
		// set q = n x p
		q[0] = a * k;
		q[1] = -n[0] * p[2];
		q[2] = n[0] * p[1];
	}
	else
	{
		// choose p in x-y plane
		b2Scalar a = n[0] * n[0] + n[1] * n[1];
		b2Scalar k = b2RecipSqrt(a);
		p[0] = -n[1] * k;
		p[1] = n[0] * k;
		p[2] = 0;
		// set q = n x p
		q[0] = -n[2] * p[1];
		q[1] = n[2] * p[0];
		q[2] = a * k;
	}
}

struct b2Vector3FloatData
{
	float m_floats[4];
};

struct b2Vector3DoubleData
{
	double m_floats[4];
};

B2_FORCE_INLINE void b2Vector3::serializeFloat(struct b2Vector3FloatData& dataOut) const
{
	///could also do a memcpy, check if it is worth it
	for (int i = 0; i < 4; i++)
		dataOut.m_floats[i] = float(m_floats[i]);
}

B2_FORCE_INLINE void b2Vector3::deSerializeFloat(const struct b2Vector3FloatData& dataIn)
{
	for (int i = 0; i < 4; i++)
		m_floats[i] = b2Scalar(dataIn.m_floats[i]);
}

B2_FORCE_INLINE void b2Vector3::serializeDouble(struct b2Vector3DoubleData& dataOut) const
{
	///could also do a memcpy, check if it is worth it
	for (int i = 0; i < 4; i++)
		dataOut.m_floats[i] = double(m_floats[i]);
}

B2_FORCE_INLINE void b2Vector3::deSerializeDouble(const struct b2Vector3DoubleData& dataIn)
{
	for (int i = 0; i < 4; i++)
		m_floats[i] = b2Scalar(dataIn.m_floats[i]);
}

B2_FORCE_INLINE void b2Vector3::serialize(struct b2Vector3Data& dataOut) const
{
	///could also do a memcpy, check if it is worth it
	for (int i = 0; i < 4; i++)
		dataOut.m_floats[i] = m_floats[i];
}

B2_FORCE_INLINE void b2Vector3::deSerialize(const struct b2Vector3Data& dataIn)
{
	for (int i = 0; i < 4; i++)
		m_floats[i] = dataIn.m_floats[i];
}

inline b2Vector3 b2MakeVector3(b2Scalar x, b2Scalar y, b2Scalar z)
{
	b2Vector3 tmp;
	tmp.setValue(x, y, z);
	return tmp;
}

inline b2Vector3 b2MakeVector3(b2Scalar x, b2Scalar y, b2Scalar z, b2Scalar w)
{
	b2Vector3 tmp;
	tmp.setValue(x, y, z);
	tmp.w = w;
	return tmp;
}

inline b2Vector4 b2MakeVector4(b2Scalar x, b2Scalar y, b2Scalar z, b2Scalar w)
{
	b2Vector4 tmp;
	tmp.setValue(x, y, z, w);
	return tmp;
}

#if defined(B2_USE_SSE_IN_API) && defined(B2_USE_SSE)

inline b2Vector3 b2MakeVector3(b2SimdFloat4 v)
{
	b2Vector3 tmp;
	tmp.set128(v);
	return tmp;
}

inline b2Vector4 b2MakeVector4(b2SimdFloat4 vec)
{
	b2Vector4 tmp;
	tmp.set128(vec);
	return tmp;
}

#endif

#endif  //B2_VECTOR3_H
