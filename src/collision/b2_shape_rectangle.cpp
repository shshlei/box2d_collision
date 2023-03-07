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

#include "box2d_collision/b2_shape_rectangle.h"
#include "box2d_collision/b2_block_allocator.h"

#include <new>

b2Shape* b2RectangleShape::Clone(b2BlockAllocator* allocator) const
{
    void* mem = allocator->Allocate(sizeof(b2RectangleShape));
    b2RectangleShape* clone = new (mem) b2RectangleShape;
    *clone = *this;
    return clone;
}

bool b2RectangleShape::TestPoint(const b2Transform& transform, const b2Vec2& p) const
{
    b2Vec2 pLocal = b2MulT(transform.q, p - transform.p);
    return b2Abs(pLocal.x) <= m_hsides(0) && b2Abs(pLocal.y) <= m_hsides(1);
}

void b2RectangleShape::ComputeAABB(b2AABB* aabb, const b2Transform& transform) const
{
    b2Scalar c = b2Abs(transform.q.c), s = b2Abs(transform.q.s);
    b2Scalar x = c * m_hsides(0) + s * m_hsides(1); 
    b2Scalar y = s * m_hsides(0) + c * m_hsides(1);
    b2Vec2 r(x, y);
    aabb->lowerBound = transform.p - r;
    aabb->upperBound = transform.p + r;
}

bool b2RectangleShape::InscribedSphereAtPoint(const b2Vec2& /*inp*/, const b2Vec2& /*bdp*/, const b2Vec2& /*normal*/, b2Vec2& local_center, b2Scalar &radius) const
{
    return false;
}

b2Vec2 b2RectangleShape::SupportPoint(const b2Vec2& dir) const
{
    b2Scalar x = dir.x >= b2Scalar(0.0) ? m_hsides(0) : -m_hsides(0);
    b2Scalar y = dir.y >= b2Scalar(0.0) ? m_hsides(1) : -m_hsides(1);
    return b2Vec2(x, y);
}
