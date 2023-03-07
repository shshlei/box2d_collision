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
#include "box2d_collision/b2_shape_ellipse.h"
#include "box2d_collision/b2_block_allocator.h"

#include <new>

b2Shape* b2EllipseShape::Clone(b2BlockAllocator* allocator) const
{
    void* mem = allocator->Allocate(sizeof(b2EllipseShape));
    b2EllipseShape* clone = new (mem) b2EllipseShape;
    *clone = *this;
    return clone;
}

bool b2EllipseShape::TestPoint(const b2Transform& transform, const b2Vec2& p) const
{
    b2Vec2 pLocal = b2MulT(transform.q, p - transform.p);
    b2Scalar d = pLocal.x * pLocal.x / m_a2 + pLocal.y * pLocal.y / m_b2;
    return d <= b2Scalar(1.0);
}

void b2EllipseShape::ComputeAABB(b2AABB* aabb, const b2Transform& transform) const
{
    b2Scalar c2 = transform.q.c * transform.q.c;
    b2Scalar s2 = transform.q.s * transform.q.s;
    b2Scalar x = b2Sqrt(c2 * m_a2 + s2 * m_b2);
    b2Scalar y = b2Sqrt(s2 * m_a2 + c2 * m_b2);
    aabb->lowerBound = transform.p - b2Vec2(x, y);
    aabb->upperBound = transform.p + b2Vec2(x, y);
}

bool b2EllipseShape::InscribedSphereAtPoint(const b2Vec2& /*inp*/, const b2Vec2& /*bdp*/, const b2Vec2& /*normal*/, b2Vec2& local_center, b2Scalar &radius) const
{
    return false;
}

b2Vec2 b2EllipseShape::SupportPoint(const b2Vec2& dir) const
{
    b2Vec2 point(m_a2 * dir.x, m_b2 * dir.y);
    point /= b2Sqrt(b2Dot(dir, point));
    return point;
}
