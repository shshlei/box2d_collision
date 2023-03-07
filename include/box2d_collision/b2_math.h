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

#include <math.h>
#include "b2_common.h"

/// This function is used to ensure that a b2Scalar point number is not a NaN or infinity.
B2_FORCE_INLINE bool b2IsValid(b2Scalar x)
{
    return isfinite(x);
}

/// A 2D column vector.
struct B2_API b2Vec2
{
    /// Default constructor does nothing (for performance).
    B2_FORCE_INLINE b2Vec2() = default;

    /// Construct using coordinates.
    B2_FORCE_INLINE b2Vec2(b2Scalar xIn, b2Scalar yIn) : x(xIn), y(yIn) {}

    B2_FORCE_INLINE b2Vec2(const b2Vec2& other) : x(other.x), y(other.y) {}

    /// Set this vector to all zeros.
    B2_FORCE_INLINE void SetZero() { x = b2Scalar(0.0); y = b2Scalar(0.0); }

    /// Set this vector to some specified coordinates.
    B2_FORCE_INLINE void Set(b2Scalar x_, b2Scalar y_) { x = x_; y = y_; }

    /// Negate this vector.
    B2_FORCE_INLINE b2Vec2 operator- () const
    {
        return b2Vec2(-x, -y);
    }

    /// Read from and indexed element.
    B2_FORCE_INLINE b2Scalar operator () (int i) const
    {
        return (&x)[i];
    }

    /// Write to an indexed element.
    B2_FORCE_INLINE b2Scalar& operator () (int i)
    {
        return (&x)[i];
    }

    B2_FORCE_INLINE b2Scalar operator [] (int i) const
    {
        return (&x)[i];
    }

    /// Write to an indexed element.
    B2_FORCE_INLINE b2Scalar& operator [] (int i)
    {
        return (&x)[i];
    }

    B2_FORCE_INLINE void operator = (const b2Vec2& v)
    {
        x = v.x; y = v.y;
    }

    /// Add a vector to this vector.
    B2_FORCE_INLINE void operator += (const b2Vec2& v)
    {
        x += v.x; y += v.y;
    }

    /// Subtract a vector from this vector.
    B2_FORCE_INLINE void operator -= (const b2Vec2& v)
    {
        x -= v.x; y -= v.y;
    }

    /// Multiply this vector by a scalar.
    B2_FORCE_INLINE void operator *= (b2Scalar a)
    {
        x *= a; y *= a;
    }

    /// Divide this vector by a scalar.
    B2_FORCE_INLINE void operator /= (b2Scalar a)
    {
        x /= a; y /= a;
    }

    /// Get the length of this vector (the norm).
    B2_FORCE_INLINE b2Scalar Length() const
    {
        return b2Sqrt(x * x + y * y);
    }

    /// Get the length squared. For performance, use this instead of
    /// b2Vec2::Length (if possible).
    B2_FORCE_INLINE b2Scalar LengthSquared() const
    {
        return x * x + y * y;
    }

    /// Get the norm of this vector (the norm).
    B2_FORCE_INLINE b2Scalar Norm() const
    {
        return b2Sqrt(x * x + y * y);
    }

    /// Get the norm squared. For performance, use this instead of
    /// b2Vec2::Length (if possible).
    B2_FORCE_INLINE b2Scalar NormSquared() const
    {
        return x * x + y * y;
    }

    /// Convert this vector into a unit vector. Returns the length.
    B2_FORCE_INLINE b2Scalar Normalize()
    {
        b2Scalar length = LengthSquared();
        if (length < b2_epsilon2)
            return b2Scalar(0.0);
        b2Scalar invLength = b2Scalar(1.0) / b2Sqrt(length);
        x *= invLength;
        y *= invLength;
        return length;
    }

    B2_FORCE_INLINE b2Vec2 Normalized()
    {
        b2Vec2 out;
        b2Scalar length = LengthSquared();
        if (length < b2_epsilon2)
            return out;
        b2Scalar invLength = b2Scalar(1.0) / b2Sqrt(length);
        out.Set(x * invLength, y * invLength);
        return out;
    }

    /// Does this vector contain finite coordinates?
    B2_FORCE_INLINE bool IsValid() const
    {
        return b2IsValid(x) && b2IsValid(y);
    }

    /// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
    B2_FORCE_INLINE b2Vec2 Skew() const
    {
        return b2Vec2(-y, x);
    }

    /// Does this vector is the zero vector?
    bool IsZero() const;

    /// Does this vector is equal to v?
    bool IsApprox(const b2Vec2& v, b2Scalar tol) const;

    b2Scalar x{0.0}, y{0.0};
};

/// A 2D column vector with 3 elements.
struct B2_API b2Vec3
{
    /// Default constructor does nothing (for performance).
    B2_FORCE_INLINE b2Vec3() = default;

    /// Construct using coordinates.
    B2_FORCE_INLINE b2Vec3(b2Scalar xIn, b2Scalar yIn, b2Scalar zIn) : x(xIn), y(yIn), z(zIn) {}

    B2_FORCE_INLINE b2Vec3(const b2Vec3& v) : x(v.x), y(v.y), z(v.z) {}

    /// Set this vector to all zeros.
    B2_FORCE_INLINE void SetZero() { x = b2Scalar(0.0); y = b2Scalar(0.0); z = b2Scalar(0.0); }

    /// Set this vector to some specified coordinates.
    B2_FORCE_INLINE void Set(b2Scalar x_, b2Scalar y_, b2Scalar z_) { x = x_; y = y_; z = z_; }

    /// Negate this vector.
    B2_FORCE_INLINE b2Vec3 operator -() const { b2Vec3 v; v.Set(-x, -y, -z); return v; }

    B2_FORCE_INLINE void operator = (const b2Vec3& v)
    {
        x = v.x; y = v.y; z = v.z;
    }

    /// Add a vector to this vector.
    B2_FORCE_INLINE void operator += (const b2Vec3& v)
    {
        x += v.x; y += v.y; z += v.z;
    }

    /// Subtract a vector from this vector.
    B2_FORCE_INLINE void operator -= (const b2Vec3& v)
    {
        x -= v.x; y -= v.y; z -= v.z;
    }

    /// Multiply this vector by a scalar.
    B2_FORCE_INLINE void operator *= (b2Scalar s)
    {
        x *= s; y *= s; z *= s;
    }

    /// Divide this vector by a scalar.
    B2_FORCE_INLINE void operator /= (b2Scalar s)
    {
        x /= s; y /= s; z /= s;
    }

    bool IsZero() const;

    bool IsApprox(const b2Vec3& v, b2Scalar tol) const;

    b2Scalar x{0.0}, y{0.0}, z{0.0};
};

/// A 2-by-2 matrix. Stored in column-major order.
struct B2_API b2Mat22
{
    /// The default constructor does nothing (for performance).
    B2_FORCE_INLINE b2Mat22() = default;

    /// Construct this matrix using columns.
    B2_FORCE_INLINE b2Mat22(const b2Vec2& c1, const b2Vec2& c2)
    {
        ex = c1;
        ey = c2;
    }

    /// Construct this matrix using scalars.
    B2_FORCE_INLINE b2Mat22(b2Scalar a11, b2Scalar a12, b2Scalar a21, b2Scalar a22)
    {
        ex.x = a11; ex.y = a21;
        ey.x = a12; ey.y = a22;
    }

    B2_FORCE_INLINE b2Mat22(const b2Mat22& v) : ex(v.ex), ey(v.ey) {}

    /// Initialize this matrix using columns.
    B2_FORCE_INLINE void Set(const b2Vec2& c1, const b2Vec2& c2)
    {
        ex = c1;
        ey = c2;
    }

    /// Set this to the identity matrix.
    B2_FORCE_INLINE void SetIdentity()
    {
        ex.x = b2Scalar(1.0); ey.x = b2Scalar(0.0);
        ex.y = b2Scalar(0.0); ey.y = b2Scalar(1.0);
    }

    /// Set this matrix to all zeros.
    B2_FORCE_INLINE void SetZero()
    {
        ex.x = b2Scalar(0.0); ey.x = b2Scalar(0.0);
        ex.y = b2Scalar(0.0); ey.y = b2Scalar(0.0);
    }

    B2_FORCE_INLINE void operator = (const b2Mat22& v)
    {
        ex = v.ex; ey = v.ey;
    }

    B2_FORCE_INLINE b2Mat22 GetInverse() const
    {
        b2Scalar a = ex.x, b = ey.x, c = ex.y, d = ey.y;
        b2Mat22 B;
        b2Scalar det = a * d - b * c;
        if (det != b2Scalar(0.0))
            det = b2Scalar(1.0) / det;
        B.ex.x =  det * d;	B.ey.x = -det * b;
        B.ex.y = -det * c;	B.ey.y =  det * a;
        return B;
    }

    /// Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases.
    B2_FORCE_INLINE b2Vec2 Solve(const b2Vec2& b) const
    {
        b2Scalar a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
        b2Scalar det = a11 * a22 - a12 * a21;
        if (det != b2Scalar(0.0))
            det = b2Scalar(1.0) / det;
        b2Vec2 x;
        x.x = det * (a22 * b.x - a12 * b.y);
        x.y = det * (a11 * b.y - a21 * b.x);
        return x;
    }

    bool IsZero() const;

    b2Vec2 ex, ey;
};

/// A 3-by-3 matrix. Stored in column-major order.
struct B2_API b2Mat33
{
    /// The default constructor does nothing (for performance).
    B2_FORCE_INLINE b2Mat33() = default;

    /// Construct this matrix using columns.
    B2_FORCE_INLINE b2Mat33(const b2Vec3& c1, const b2Vec3& c2, const b2Vec3& c3)
    {
        ex = c1;
        ey = c2;
        ez = c3;
    }

    B2_FORCE_INLINE b2Mat33(b2Scalar a11, b2Scalar a12, b2Scalar a13,
            b2Scalar a21, b2Scalar a22, b2Scalar a23,
            b2Scalar a31, b2Scalar a32, b2Scalar a33)
    {
        ex.x = a11; ex.y = a21; ex.z = a31;
        ey.x = a12; ey.y = a22; ey.z = a32;
        ez.x = a13; ez.y = a23; ez.z = a33;
    }

    B2_FORCE_INLINE b2Mat33(const b2Mat33& v) : ex(v.ex), ey(v.ey), ez(v.ez) {} 

    /// Set this matrix to all zeros.
    B2_FORCE_INLINE void SetZero()
    {
        ex.SetZero();
        ey.SetZero();
        ez.SetZero();
    }

    B2_FORCE_INLINE void operator = (const b2Mat33& v)
    {
        ex = v.ex; ey = v.ey; ez = v.ez;
    }

    /// Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases.
    B2_FORCE_INLINE b2Vec3 Solve33(const b2Vec3& b) const;

    /// Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases. Solve only the upper
    /// 2-by-2 matrix equation.
    B2_FORCE_INLINE b2Vec2 Solve22(const b2Vec2& b) const;

    /// Get the inverse of this matrix as a 2-by-2.
    /// Returns the zero matrix if singular.
    B2_FORCE_INLINE void GetInverse22(b2Mat33* M) const;

    /// Get the symmetric inverse of this matrix as a 3-by-3.
    /// Returns the zero matrix if singular.
    B2_FORCE_INLINE void GetSymInverse33(b2Mat33* M) const;

    bool IsZero() const;

    b2Vec3 ex, ey, ez;
};

/// Rotation
struct B2_API b2Rot
{
    B2_FORCE_INLINE b2Rot() = default;

    /// Initialize from an angle in radians
    B2_FORCE_INLINE explicit b2Rot(b2Scalar angle)
    {
        /// TODO_ERIN optimize
        s = b2Sin(angle);
        c = b2Cos(angle);
    }

    /// Set using an angle in radians.
    B2_FORCE_INLINE void Set(b2Scalar angle)
    {
        /// TODO_ERIN optimize
        s = b2Sin(angle);
        c = b2Cos(angle);
    }

    /// Set to the identity rotation
    B2_FORCE_INLINE void SetIdentity()
    {
        s = b2Scalar(0.0);
        c = b2Scalar(1.0);
    }

    B2_FORCE_INLINE b2Rot(const b2Rot& v) : s(v.s), c(v.c) {} 

    /// Get the angle in radians
    B2_FORCE_INLINE b2Scalar GetAngle() const
    {
        return b2Atan2(s, c);
    }

    /// Get the x-axis
    B2_FORCE_INLINE b2Vec2 GetXAxis() const
    {
        return b2Vec2(c, s);
    }

    /// Get the u-axis
    B2_FORCE_INLINE b2Vec2 GetYAxis() const
    {
        return b2Vec2(-s, c);
    }

    B2_FORCE_INLINE void operator = (const b2Rot& v)
    {
        s = v.s; c = v.c;;
    }

    bool IsIdentity() const;

    /// Sine and cosine
    b2Scalar s{0.0}, c{1.0};
};

/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
struct B2_API b2Transform
{
    /// The default constructor does nothing.
    B2_FORCE_INLINE b2Transform() = default;

    /// Initialize using a position vector and a rotation.
    B2_FORCE_INLINE b2Transform(const b2Vec2& position, b2Scalar angle) : p(position), q(angle)
    {
    }

    B2_FORCE_INLINE b2Transform(b2Scalar x, b2Scalar y, b2Scalar angle) : p(x, y), q(angle)
    {
    }

    /// Initialize using a position vector and a rotation.
    B2_FORCE_INLINE b2Transform(const b2Vec2& position, const b2Rot& rotation) : p(position), q(rotation) {}

    B2_FORCE_INLINE b2Transform(const b2Transform& v) : p(v.p), q(v.q) {} 

    /// Set this to the identity transform.
    B2_FORCE_INLINE void SetIdentity()
    {
        p.SetZero();
        q.SetIdentity();
    }

    /// Set this based on the position and angle.
    B2_FORCE_INLINE void Set(const b2Vec2& position, b2Scalar angle)
    {
        p = position;
        q.Set(angle);
    }

    B2_FORCE_INLINE void operator = (const b2Transform& v)
    {
        p = v.p; q = v.q;;
    }

    B2_FORCE_INLINE b2Vec2 GetPosition() const
    {
        return p;
    }

    B2_FORCE_INLINE b2Scalar GetAngle() const
    {
        return q.GetAngle();
    }

    bool IsIdentity() const;

    b2Vec2 p;
    b2Rot q;
};

/// An axis aligned bounding box.
struct B2_API b2AABB;

/// Useful constant
extern B2_API const b2Vec2 b2Vec2_zero;

/// Perform the dot product on two vectors.
B2_FORCE_INLINE b2Scalar b2Dot(const b2Vec2& a, const b2Vec2& b)
{
    return a.x * b.x + a.y * b.y;
}

/// Perform the cross product on two vectors. In 2D this produces a scalar.
B2_FORCE_INLINE b2Scalar b2Cross(const b2Vec2& a, const b2Vec2& b)
{
    return a.x * b.y - a.y * b.x;
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
B2_FORCE_INLINE b2Vec2 b2Cross(const b2Vec2& a, b2Scalar s)
{
    return b2Vec2(s * a.y, -s * a.x);
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
B2_FORCE_INLINE b2Vec2 b2Cross(b2Scalar s, const b2Vec2& a)
{
    return b2Vec2(-s * a.y, s * a.x);
}

/// Multiply a matrix times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another.
B2_FORCE_INLINE b2Vec2 b2Mul(const b2Mat22& A, const b2Vec2& v)
{
    return b2Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
}

/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
B2_FORCE_INLINE b2Vec2 b2MulT(const b2Mat22& A, const b2Vec2& v)
{
    return b2Vec2(b2Dot(v, A.ex), b2Dot(v, A.ey));
}

/// Add two vectors component-wise.
B2_FORCE_INLINE b2Vec2 operator + (const b2Vec2& a, const b2Vec2& b)
{
    return b2Vec2(a.x + b.x, a.y + b.y);
}

/// Subtract two vectors component-wise.
B2_FORCE_INLINE b2Vec2 operator - (const b2Vec2& a, const b2Vec2& b)
{
    return b2Vec2(a.x - b.x, a.y - b.y);
}

B2_FORCE_INLINE b2Vec2 operator * (b2Scalar s, const b2Vec2& a)
{
    return b2Vec2(s * a.x, s * a.y);
}

B2_FORCE_INLINE b2Vec2 operator * (const b2Vec2& a, b2Scalar s)
{
    return b2Vec2(s * a.x, s * a.y);
}

B2_FORCE_INLINE b2Vec2 operator / (const b2Vec2& v, b2Scalar a)
{
    return b2Vec2(v.x / a, v.y / a);
}

B2_FORCE_INLINE bool operator == (const b2Vec2& a, const b2Vec2& b)
{
    return a.x == b.x && a.y == b.y;
}

B2_FORCE_INLINE bool operator != (const b2Vec2& a, const b2Vec2& b)
{
    return a.x != b.x || a.y != b.y;
}

B2_FORCE_INLINE b2Scalar b2Distance(const b2Vec2& a, const b2Vec2& b)
{
    b2Vec2 c = a - b;
    return c.Length();
}

B2_FORCE_INLINE b2Scalar b2DistanceSquared(const b2Vec2& a, const b2Vec2& b)
{
    b2Vec2 c = a - b;
    return b2Dot(c, c);
}

B2_FORCE_INLINE b2Vec3 operator * (b2Scalar s, const b2Vec3& a)
{
    return b2Vec3(s * a.x, s * a.y, s * a.z);
}

B2_FORCE_INLINE b2Vec3 operator * (const b2Vec3& a, b2Scalar s)
{
    return b2Vec3(s * a.x, s * a.y, s * a.z);
}

B2_FORCE_INLINE b2Vec3 operator / (const b2Vec3& a, b2Scalar s)
{
    return b2Vec3(a.x / s, a.y / s, a.z / s);
}

/// Add two vectors component-wise.
B2_FORCE_INLINE b2Vec3 operator + (const b2Vec3& a, const b2Vec3& b)
{
    return b2Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

/// Subtract two vectors component-wise.
B2_FORCE_INLINE b2Vec3 operator - (const b2Vec3& a, const b2Vec3& b)
{
    return b2Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

/// Perform the dot product on two vectors.
B2_FORCE_INLINE b2Scalar b2Dot(const b2Vec3& a, const b2Vec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Perform the cross product on two vectors.
B2_FORCE_INLINE b2Vec3 b2Cross(const b2Vec3& a, const b2Vec3& b)
{
    return b2Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

B2_FORCE_INLINE b2Mat22 operator + (const b2Mat22& A, const b2Mat22& B)
{
    return b2Mat22(A.ex + B.ex, A.ey + B.ey);
}

// A * B
B2_FORCE_INLINE b2Mat22 b2Mul(const b2Mat22& A, const b2Mat22& B)
{
    return b2Mat22(b2Mul(A, B.ex), b2Mul(A, B.ey));
}

// A^T * B
B2_FORCE_INLINE b2Mat22 b2MulT(const b2Mat22& A, const b2Mat22& B)
{
    b2Vec2 c1(b2Dot(A.ex, B.ex), b2Dot(A.ey, B.ex));
    b2Vec2 c2(b2Dot(A.ex, B.ey), b2Dot(A.ey, B.ey));
    return b2Mat22(c1, c2);
}

/// Multiply a matrix times a vector.
B2_FORCE_INLINE b2Vec3 b2Mul(const b2Mat33& A, const b2Vec3& v)
{
    return v.x * A.ex + v.y * A.ey + v.z * A.ez;
}

/// Multiply a matrix times a vector.
B2_FORCE_INLINE b2Vec2 b2Mul22(const b2Mat33& A, const b2Vec2& v)
{
    return b2Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
}

/// Multiply two rotations: q * r
B2_FORCE_INLINE b2Rot b2Mul(const b2Rot& q, const b2Rot& r)
{
    // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
    // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
    // s = qs * rc + qc * rs
    // c = qc * rc - qs * rs
    b2Rot qr;
    qr.s = q.s * r.c + q.c * r.s;
    qr.c = q.c * r.c - q.s * r.s;
    return qr;
}

/// Transpose multiply two rotations: qT * r
B2_FORCE_INLINE b2Rot b2MulT(const b2Rot& q, const b2Rot& r)
{
    // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
    // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
    // s = qc * rs - qs * rc
    // c = qc * rc + qs * rs
    b2Rot qr;
    qr.s = q.c * r.s - q.s * r.c;
    qr.c = q.c * r.c + q.s * r.s;
    return qr;
}

/// Rotate a vector
B2_FORCE_INLINE b2Vec2 b2Mul(const b2Rot& q, const b2Vec2& v)
{
    return b2Vec2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
}

/// Inverse rotate a vector
B2_FORCE_INLINE b2Vec2 b2MulT(const b2Rot& q, const b2Vec2& v)
{
    return b2Vec2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y);
}

B2_FORCE_INLINE b2Vec2 b2Mul(const b2Transform& T, const b2Vec2& v)
{
    b2Scalar x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
    b2Scalar y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;

    return b2Vec2(x, y);
}

B2_FORCE_INLINE b2Vec2 b2MulT(const b2Transform& T, const b2Vec2& v)
{
    b2Scalar px = v.x - T.p.x;
    b2Scalar py = v.y - T.p.y;
    b2Scalar x = (T.q.c * px + T.q.s * py);
    b2Scalar y = (-T.q.s * px + T.q.c * py);

    return b2Vec2(x, y);
}

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
B2_FORCE_INLINE b2Transform b2Mul(const b2Transform& A, const b2Transform& B)
{
    b2Transform C;
    C.q = b2Mul(A.q, B.q);
    C.p = b2Mul(A.q, B.p) + A.p;
    return C;
}

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
B2_FORCE_INLINE b2Transform b2MulT(const b2Transform& A, const b2Transform& B)
{
    b2Transform C;
    C.q = b2MulT(A.q, B.q);
    C.p = b2MulT(A.q, B.p - A.p);
    return C;
}

template <typename T>
B2_FORCE_INLINE T b2Abs(T a)
{
    return a > T(0) ? a : -a;
}

B2_FORCE_INLINE b2Vec2 b2Abs(const b2Vec2& a)
{
    return b2Vec2(b2Abs(a.x), b2Abs(a.y));
}

B2_FORCE_INLINE b2Mat22 b2Abs(const b2Mat22& A)
{
    return b2Mat22(b2Abs(A.ex), b2Abs(A.ey));
}

template <typename T>
B2_FORCE_INLINE T b2Min(T a, T b)
{
    return a < b ? a : b;
}

B2_FORCE_INLINE b2Vec2 b2Min(const b2Vec2& a, const b2Vec2& b)
{
    return b2Vec2(b2Min(a.x, b.x), b2Min(a.y, b.y));
}

template <typename T>
B2_FORCE_INLINE T b2Max(T a, T b)
{
    return a > b ? a : b;
}

B2_FORCE_INLINE b2Vec2 b2Max(const b2Vec2& a, const b2Vec2& b)
{
    return b2Vec2(b2Max(a.x, b.x), b2Max(a.y, b.y));
}

template <typename T>
B2_FORCE_INLINE T b2Clamp(T a, T low, T high)
{
    return b2Max(low, b2Min(a, high));
}

B2_FORCE_INLINE b2Vec2 b2Clamp(const b2Vec2& a, const b2Vec2& low, const b2Vec2& high)
{
    return b2Max(low, b2Min(a, high));
}

/// "Next Largest Power of 2
/// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
/// largest power of 2. For a 32-bit value:"
B2_FORCE_INLINE unsigned int b2NextPowerOfTwo(unsigned int x)
{
    x |= (x >> 1);
    x |= (x >> 2);
    x |= (x >> 4);
    x |= (x >> 8);
    x |= (x >> 16);
    return x + 1;
}

B2_FORCE_INLINE bool b2IsPowerOfTwo(unsigned int x)
{
    bool result = x > 0 && (x & (x - 1)) == 0;
    return result;
}

struct B2_API b2AABB
{
    b2AABB()
    {
        lowerBound.Set(b2_maxFloat, b2_maxFloat);
        upperBound.Set(-b2_maxFloat, -b2_maxFloat);
    }

    /// @brief Creating an b2AABB at position v with zero size
    b2AABB(const b2Vec2& v) : lowerBound(v), upperBound(v)
    {
    }

    /// @brief Creating an b2AABB with two endpoints a and b
    b2AABB(const b2Vec2& a, const b2Vec2&b) : lowerBound(b2Min(a, b)), upperBound(b2Max(a, b))
    {
    }

    /// @brief Creating an b2AABB centered as core and is of half-dimension delta
    b2AABB(const b2AABB& core, const b2Vec2& delta) : lowerBound(core.lowerBound - delta), upperBound(core.upperBound + delta)
    {
    }

    B2_FORCE_INLINE b2AABB(const b2AABB& v) : lowerBound(v.lowerBound), upperBound(v.upperBound) {} 

    B2_FORCE_INLINE void operator = (const b2AABB& v)
    {
        lowerBound = v.lowerBound;
        upperBound = v.upperBound;
    }

    /// Verify that the bounds are sorted.
    bool IsValid() const;

    B2_FORCE_INLINE void SetMin(b2Scalar min)
    {
        lowerBound.Set(min, min);
    }

    B2_FORCE_INLINE void SetMin(b2Scalar minx, b2Scalar miny)
    {
        lowerBound.Set(minx, miny);
    }

    B2_FORCE_INLINE void SetMax(b2Scalar max)
    {
        upperBound.Set(max, max);
    }

    B2_FORCE_INLINE void SetMax(b2Scalar maxx, b2Scalar maxy)
    {
        upperBound.Set(maxx, maxy);
    }

    /// Get the center of the AABB.
    B2_FORCE_INLINE b2Vec2 GetCenter() const
    {
        return b2Scalar(0.5) * (lowerBound + upperBound);
    }

    /// Get the extents of the AABB (half-widths).
    B2_FORCE_INLINE b2Vec2 GetExtents() const
    {
        return b2Scalar(0.5) * (upperBound - lowerBound);
    }

    /// @brief Width of the b2AABB
    B2_FORCE_INLINE b2Scalar Width() const
    {
        return upperBound.x - lowerBound.x;
    }

    /// @brief Height of the b2AABB
    B2_FORCE_INLINE b2Scalar Height() const
    {
        return upperBound.y - lowerBound.y;
    }

    /// @brief Area of the b2AABB
    B2_FORCE_INLINE b2Scalar Area() const
    {
        return Width() * Height();
    }

    /// @brief Size of the b2AABB 
    B2_FORCE_INLINE b2Scalar Size() const
    {
        return (upperBound - lowerBound).LengthSquared();
    }

    /// @brief Radius of the b2AABB
    B2_FORCE_INLINE b2Scalar Radius() const
    {
        return 0.5 * (upperBound - lowerBound).Length();
    }

    /// Get the perimeter length
    B2_FORCE_INLINE b2Scalar GetPerimeter() const
    {
        return b2Scalar(2.0) * (Width() + Height());
    }

    /// Combine an AABB into this one.
    B2_FORCE_INLINE void Combine(const b2AABB& aabb)
    {
        lowerBound = b2Min(lowerBound, aabb.lowerBound);
        upperBound = b2Max(upperBound, aabb.upperBound);
    }

    /// Combine two AABBs into this one.
    B2_FORCE_INLINE void Combine(const b2AABB& aabb1, const b2AABB& aabb2)
    {
        lowerBound = b2Min(aabb1.lowerBound, aabb2.lowerBound);
        upperBound = b2Max(aabb1.upperBound, aabb2.upperBound);
    }

    /// @brief Check whether two b2AABB are Overlap
    B2_FORCE_INLINE bool Overlap(const b2AABB& other) const
    {
        if (lowerBound.x > other.upperBound.x)
            return false;
        if (lowerBound.y > other.upperBound.y)
            return false;
        if (other.lowerBound.x > upperBound.x)
            return false;
        if (other.lowerBound.y > upperBound.y)
            return false;
        return true;
    }

    /// Does this aabb contain the provided AABB.
    B2_FORCE_INLINE bool Contains(const b2AABB& aabb) const
    {
        bool result = true;
        result = result && lowerBound.x <= aabb.lowerBound.x;
        result = result && lowerBound.y <= aabb.lowerBound.y;
        result = result && aabb.upperBound.x <= upperBound.x;
        result = result && aabb.upperBound.y <= upperBound.y;
        return result;
    }

    /// Does this aabb contain the provided point.
    B2_FORCE_INLINE bool Contains(const b2Vec2& aabb) const
    {
        if (aabb.x < lowerBound.x)
            return false;
        if (aabb.x > upperBound.x)
            return false;
        if (aabb.y < lowerBound.y)
            return false;
        if (aabb.y > upperBound.y)
            return false;
        return true;
    }

    /// @brief Check whether two b2AABB are overlapped along specific axis
    B2_FORCE_INLINE bool AxisOverlap(const b2AABB& other, int axis_id) const
    {
        if (lowerBound[axis_id] > other.upperBound[axis_id])
            return false;
        if (upperBound[axis_id] < other.lowerBound[axis_id])
            return false;
        return true;
    }

    /// @brief Check whether two b2AABB are Overlap and return the Overlap part
    B2_FORCE_INLINE bool Overlap(const b2AABB& other, b2AABB& overlap_part) const
    {
        if (!Overlap(other))
            return false;
        overlap_part.lowerBound = b2Max(lowerBound, other.lowerBound);
        overlap_part.upperBound = b2Min(upperBound, other.upperBound);
        return true;
    }

    /// @brief Merge the b2AABB and a point
    B2_FORCE_INLINE b2AABB& operator += (const b2Vec2& p)
    {
        lowerBound = b2Min(lowerBound, p);
        upperBound = b2Max(upperBound, p);
        return *this;
    }

    /// @brief Merge the b2AABB and another b2AABB
    B2_FORCE_INLINE b2AABB& operator += (const b2AABB& other)
    {
        lowerBound = b2Min(lowerBound, other.lowerBound);
        upperBound = b2Max(upperBound, other.upperBound);
        return *this;
    }

    /// @brief Return the merged b2AABB of current b2AABB and the other one
    B2_FORCE_INLINE b2AABB operator + (const b2AABB& other) const
    {
        b2AABB res(*this);
        return res += other;
    }

    /// @brief Distance between two b2AABBs; P and Q, should not be nullptr, return
    /// the nearest points
    b2Scalar Distance(const b2AABB& other, b2Vec2* P, b2Vec2* Q) const
    {
        b2Scalar result = 0.0;
        for (int i = 0; i < 2; ++i)
        {
            const b2Scalar& amin = lowerBound[i];
            const b2Scalar& amax = upperBound[i];
            const b2Scalar& bmin = other.lowerBound[i];
            const b2Scalar& bmax = other.upperBound[i];
            if (amin > bmax)
            {
                b2Scalar delta = amin - bmax;
                result += delta * delta;
                if (P && Q)
                {
                    (*P)[i] = amin;
                    (*Q)[i] = bmax;
                }
            }
            else if (bmin > amax)
            {
                b2Scalar delta = bmin - amax;
                result += delta * delta;
                if (P && Q)
                {
                    (*P)[i] = amax;
                    (*Q)[i] = bmin;
                }               
            }
            else if (P && Q)
            {
                if (bmin >= amin)
                {
                    b2Scalar t = 0.5 * (amax + bmin);
                    (*P)[i] = t;
                    (*Q)[i] = t;
                }
                else
                {
                    b2Scalar t = 0.5 * (amin + bmax);
                    (*P)[i] = t;
                    (*Q)[i] = t;
                }
            }
        }
        return b2Sqrt(result);
    }

    /// @brief Distance between two b2AABBs
    b2Scalar Distance(const b2AABB& other) const
    {
        b2Scalar result = 0.0;
        for (int i = 0; i < 2; ++i)
        {
            const b2Scalar& amin = lowerBound[i];
            const b2Scalar& amax = upperBound[i];
            const b2Scalar& bmin = other.lowerBound[i];
            const b2Scalar& bmax = other.upperBound[i];
            if (amin > bmax)
            {
                b2Scalar delta = amin - bmax;
                result += delta * delta;
            }
            else if (bmin > amax)
            {
                b2Scalar delta = bmin - amax;
                result += delta * delta;
            }
        }
        return b2Sqrt(result);
    }

    b2Scalar Distance(const b2Vec2& point) const
    {
        b2Scalar result = 0.0;
        for (int i = 0; i < 2; ++i)
        {
            const b2Scalar& amin = lowerBound[i];
            const b2Scalar& amax = upperBound[i];
            const b2Scalar& bmin = point[i];
            const b2Scalar& bmax = point[i];
            if (amin > bmax)
            {
                b2Scalar delta = amin - bmax;
                result += delta * delta;
            }
            else if (bmin > amax)
            {
                b2Scalar delta = bmin - amax;
                result += delta * delta;
            }
        }
        return b2Sqrt(result);
    }

    /// @brief whether two b2AABB are equal
    B2_FORCE_INLINE bool Equal(const b2AABB& other) const
    {
        return lowerBound.IsApprox(other.lowerBound, 100.0 * b2_epsilon) && upperBound.IsApprox(other.upperBound, 100.0 * b2_epsilon);
    }

    /// @brief expand the half size of the b2AABB by delta, and keep the center unchanged.
    B2_FORCE_INLINE b2AABB& Expand(const b2Vec2& delta)
    {
        lowerBound -= delta;
        upperBound += delta;
        return *this;
    }

    B2_FORCE_INLINE b2AABB Translate(const b2AABB& aabb, const b2Vec2& t)
    {
        b2AABB res(aabb);
        res.lowerBound += t;
        res.upperBound += t;
        return res;
    }

    b2Vec2 lowerBound;	///< the lower vertex
    b2Vec2 upperBound;	///< the upper vertex
};

B2_FORCE_INLINE bool b2AABB::IsValid() const
{
    b2Vec2 d = upperBound - lowerBound;
    bool valid = d.x >= b2Scalar(0.0) && d.y >= b2Scalar(0.0);
    valid = valid && lowerBound.IsValid() && upperBound.IsValid();
    return valid;
}

B2_FORCE_INLINE bool b2TestOverlap(const b2AABB& a, const b2AABB& b)
{
    b2Vec2 d1, d2;
    d1 = b.lowerBound - a.upperBound;
    d2 = a.lowerBound - b.upperBound;
    if (d1.x > b2Scalar(0.0) || d1.y > b2Scalar(0.0))
        return false;
    if (d2.x > b2Scalar(0.0) || d2.y > b2Scalar(0.0))
        return false;
    return true;
}

B2_FORCE_INLINE bool b2Vec2::IsZero() const
{
    return b2Abs(x) < b2_epsilon && b2Abs(y) < b2_epsilon;
}

B2_FORCE_INLINE bool b2Vec3::IsZero() const
{
    return b2Abs(x) < b2_epsilon && b2Abs(y) < b2_epsilon && b2Abs(z) < b2_epsilon;
}

B2_FORCE_INLINE bool b2Vec2::IsApprox(const b2Vec2& v, b2Scalar tol) const
{
    return b2Abs(x - v.x) < tol && b2Abs(y - v.y) < tol;
}

B2_FORCE_INLINE bool b2Vec3::IsApprox(const b2Vec3& v, b2Scalar tol) const
{
    return b2Abs(x - v.x) < tol && b2Abs(y - v.y) < tol && b2Abs(z - v.z) < tol;
}

B2_FORCE_INLINE bool b2Mat22::IsZero() const
{
    return ex.IsZero() && ey.IsZero();
}

B2_FORCE_INLINE bool b2Mat33::IsZero() const
{
    return ex.IsZero() && ey.IsZero() && ez.IsZero();
}

B2_FORCE_INLINE bool b2Rot::IsIdentity() const
{
    return c > b2Scalar(1.0) - b2_epsilon;
}

B2_FORCE_INLINE bool b2Transform::IsIdentity() const
{
    return p.IsZero() && q.IsIdentity();
}

#endif
