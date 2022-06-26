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

#include "box2d_collision/b2_math.h"

const b2Vec2 b2Vec2_zero(b2Scalar(0.0), b2Scalar(0.0));

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
B2_FORCE_INLINE b2Vec3 b2Mat33::Solve33(const b2Vec3& b) const
{
	b2Scalar det = b2Dot(ex, b2Cross(ey, ez));
	if (det != b2Scalar(0.0))
	{
		det = b2Scalar(1.0) / det;
	}
	b2Vec3 x;
	x.x = det * b2Dot(b, b2Cross(ey, ez));
	x.y = det * b2Dot(ex, b2Cross(b, ez));
	x.z = det * b2Dot(ex, b2Cross(ey, b));
	return x;
}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
B2_FORCE_INLINE b2Vec2 b2Mat33::Solve22(const b2Vec2& b) const
{
	b2Scalar a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
	b2Scalar det = a11 * a22 - a12 * a21;
	if (det != b2Scalar(0.0))
	{
		det = b2Scalar(1.0) / det;
	}
	b2Vec2 x;
	x.x = det * (a22 * b.x - a12 * b.y);
	x.y = det * (a11 * b.y - a21 * b.x);
	return x;
}

///
B2_FORCE_INLINE void b2Mat33::GetInverse22(b2Mat33* M) const
{
	b2Scalar a = ex.x, b = ey.x, c = ex.y, d = ey.y;
	b2Scalar det = a * d - b * c;
	if (det != b2Scalar(0.0))
	{
		det = b2Scalar(1.0) / det;
	}

	M->ex.x =  det * d;	M->ey.x = -det * b; M->ex.z = b2Scalar(0.0);
	M->ex.y = -det * c;	M->ey.y =  det * a; M->ey.z = b2Scalar(0.0);
	M->ez.x = b2Scalar(0.0); M->ez.y = b2Scalar(0.0); M->ez.z = b2Scalar(0.0);
}

/// Returns the zero matrix if singular.
B2_FORCE_INLINE void b2Mat33::GetSymInverse33(b2Mat33* M) const
{
	b2Scalar det = b2Dot(ex, b2Cross(ey, ez));
	if (det != b2Scalar(0.0))
	{
		det = b2Scalar(1.0) / det;
	}

	b2Scalar a11 = ex.x, a12 = ey.x, a13 = ez.x;
	b2Scalar a22 = ey.y, a23 = ez.y;
	b2Scalar a33 = ez.z;

	M->ex.x = det * (a22 * a33 - a23 * a23);
	M->ex.y = det * (a13 * a23 - a12 * a33);
	M->ex.z = det * (a12 * a23 - a13 * a22);

	M->ey.x = M->ex.y;
	M->ey.y = det * (a11 * a33 - a13 * a13);
	M->ey.z = det * (a13 * a12 - a11 * a23);

	M->ez.x = M->ex.z;
	M->ez.y = M->ey.z;
	M->ez.z = det * (a11 * a22 - a12 * a12);
}
