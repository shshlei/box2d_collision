// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef B2_MATH_H
#define B2_MATH_H

#include <cfloat>

#define B2_USE_DOUBLE_PRECISION
/// The b2Scalar type abstracts floating point numbers, to easily switch between double and single floating point precision.
#if defined(B2_USE_DOUBLE_PRECISION)
typedef double b2Scalar;
// this number could be bigger in double precision
#define B2_LARGE_FLOAT 1e30
#else
typedef float b2Scalar;
// keep B2_LARGE_FLOAT*B2_LARGE_FLOAT < FLT_MAX
#define B2_LARGE_FLOAT 1e18f
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

#define B2_EPSILON_2 B2_EPSILON * B2_EPSILON

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef Eigen::Matrix<b2Scalar, 2, 1> b2Vec2;
typedef Eigen::Matrix<b2Scalar, 3, 1> b2Vec3;
typedef Eigen::Matrix<b2Scalar, 2, 2> b2Mat22;
typedef Eigen::Matrix<b2Scalar, 3, 3> b2Mat33;
typedef Eigen::Rotation2D<b2Scalar> b2Rot;
typedef Eigen::Transform<b2Scalar, 2, Eigen::Isometry> b2Transform;
typedef Eigen::AlignedBox<b2Scalar, 2> b2AABB;

class b2OBB
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  b2OBB() = default;

  b2OBB(const b2Mat22 & axis, const b2Vec2 & center, const b2Vec2 & extent);

  // Axis of the OBB
  const b2Mat22 & axis() const;

  // Axis of the OBB
  const b2Vec2 & center() const;

  // Extent of the OBB
  const b2Vec2 & extent() const;

  // Width of the OBB.
  b2Scalar width() const;

  // Height of the OBB.
  b2Scalar height() const;

  // Volume of the OBB
  b2Scalar volume() const;

  // Size of the OBB
  b2Scalar size() const;

  // Check whether the OBB contains a point.
  bool contain(const b2Vec2 & p) const;

  // Check collision between two OBB.
  bool overlap(const b2OBB & other) const;

  // Distance between two OBBs, not implemented.
  // b2Scalar distance(const b2OBB & other, b2Vec2 * P = nullptr, b2Vec2 * Q = nullptr) const;

private:
  b2Mat22 axis_;

  // Center of OBB
  b2Vec2 center_;

  // Half dimensions of OBB
  b2Vec2 extent_;
};

void computeVertices(const b2OBB & b, b2Vec2 vertices[4]);

// B rotation, T translation
// a extent, b extent
bool obbDisjoint(const b2Mat22 & B, const b2Vec2 & T, const b2Vec2 & a, const b2Vec2 & b);

/// Perform the dot product on two vectors.
inline b2Scalar b2Dot(const b2Vec2 & a, const b2Vec2 & b)
{
  return a.dot(b);
}

/// Perform the cross product on two vectors. In 2D this produces a scalar.
inline b2Scalar b2Cross(const b2Vec2 & a, const b2Vec2 & b)
{
  return a.x() * b.y() - a.y() * b.x();
}

/// Perform the cross product on a vector and a scalar. In 2D this produces a vector.
inline b2Vec2 b2Cross(const b2Vec2 & a, b2Scalar s)
{
  return b2Vec2(s * a.y(), -s * a.x());
}

/// Perform the cross product on a scalar and a vector. In 2D this produces a vector.
inline b2Vec2 b2Cross(b2Scalar s, const b2Vec2 & a)
{
  return b2Vec2(-s * a.y(), s * a.x());
}

/// Multiply a matrix times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another.
inline b2Vec2 b2Mul(const b2Mat22 & A, const b2Vec2 & v)
{
  return A * v;
}

/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
inline b2Vec2 b2MulT(const b2Mat22 & A, const b2Vec2 & v)
{
  return A.transpose() * v;
}

inline b2Scalar b2Distance(const b2Vec2 & a, const b2Vec2 & b)
{
  return (a - b).norm();
}

inline b2Scalar b2DistanceSquared(const b2Vec2 & a, const b2Vec2 & b)
{
  return (a - b).squaredNorm();
}

// A * B
inline b2Mat22 b2Mul(const b2Mat22 & A, const b2Mat22 & B)
{
  return A * B;
}

// A^T * B
inline b2Mat22 b2MulT(const b2Mat22 & A, const b2Mat22 & B)
{
  return A.transpose() * B;
}

/// Multiply two rotations: q * r
inline b2Rot b2Mul(const b2Rot & q, const b2Rot & r)
{
  return q * r;
}

/// Transpose multiply two rotations: qT * r
inline b2Rot b2MulT(const b2Rot & q, const b2Rot & r)
{
  return q.inverse() * r;
}

/// Rotate a vector
inline b2Vec2 b2Mul(const b2Rot & q, const b2Vec2 & v)
{
  return q * v;
}

/// Inverse rotate a vector
inline b2Vec2 b2MulT(const b2Rot & q, const b2Vec2 & v)
{
  return q.inverse() * v;
}

inline b2Vec2 b2Mul(const b2Transform & T, const b2Vec2 & v)
{
  return T * v;
}

inline b2Vec2 b2MulT(const b2Transform & T, const b2Vec2 & v)
{
  return T.inverse() * v;
}

inline b2Transform b2Mul(const b2Transform & A, const b2Transform & B)
{
  return A * B;
}

inline b2Transform b2MulT(const b2Transform & A, const b2Transform & B)
{
  return A.inverse() * B;
}

template <typename T>
inline T b2Abs(T a)
{
  return (a > T(0) ? a : -a);
}

inline b2Vec2 b2Abs(const b2Vec2 & a)
{
  return a.cwiseAbs();
}

inline b2Mat22 b2Abs(const b2Mat22 & A)
{
  return A.cwiseAbs();
}

template <typename T>
inline T b2Min(T a, T b)
{
  return (a < b ? a : b);
}

inline b2Vec2 b2Min(const b2Vec2 & a, const b2Vec2 & b)
{
  return a.cwiseMin(b);
}

template <typename T>
inline T b2Max(T a, T b)
{
  return (a > b ? a : b);
}

inline b2Vec2 b2Max(const b2Vec2 & a, const b2Vec2 & b)
{
  return a.cwiseMax(b);
}

template <typename T>
inline T b2Clamp(T a, T low, T high)
{
  return b2Max(low, b2Min(a, high));
}

inline b2Vec2 b2Clamp(const b2Vec2 & a, const b2Vec2 & low, const b2Vec2 & high)
{
  return b2Max(low, b2Min(a, high));
}

/// "Next Largest Power of 2
/// Given a bina.y() integer value x, the next largest power of 2 can be computed by a SWAR algorithm
/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
/// largest power of 2. For a 32-bit value:"
inline unsigned int b2NextPowerOfTwo(unsigned int x)
{
  x |= (x >> 1);
  x |= (x >> 2);
  x |= (x >> 4);
  x |= (x >> 8);
  x |= (x >> 16);
  return x + 1;
}

inline bool b2IsPowerOfTwo(unsigned int x)
{
  bool result = x > 0 && (x & (x - 1)) == 0;
  return result;
}

template <typename BV1, typename BV2>
inline bool b2TestOverlap(const BV1 & a, const BV2 & b)
{
  return false;
}

template <>
inline bool b2TestOverlap(const b2AABB & a, const b2AABB & b)
{
  return a.intersects(b);
}

template <>
inline bool b2TestOverlap(const b2AABB & a, const b2OBB & b)
{
  return !obbDisjoint(b.axis(), b.center() - a.center(), 0.5 * a.sizes(), b.extent());
}

template <>
inline bool b2TestOverlap(const b2OBB & b, const b2AABB & a)
{
  return !obbDisjoint(b.axis(), b.center() - a.center(), 0.5 * a.sizes(), b.extent());
}

#endif
