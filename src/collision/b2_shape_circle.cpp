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

#include "box2d_collision/b2_shape_circle.h"
#include "box2d_collision/b2_block_allocator.h"

#include <new>

b2Shape* b2CircleShape::Clone(b2BlockAllocator* allocator) const
{
    void* mem = allocator->Allocate(sizeof(b2CircleShape));
    b2CircleShape* clone = new (mem) b2CircleShape;
    *clone = *this;
    return clone;
}

bool b2CircleShape::TestPoint(const b2Transform& transform, const b2Vec2& p) const
{
    b2Vec2 d = p - transform.p;
    return b2Dot(d, d) <= m_radius * m_radius;
}

void b2CircleShape::ComputeAABB(b2AABB* aabb, const b2Transform& transform) const
{
    b2Vec2 r(m_radius, m_radius);
    aabb->lowerBound = transform.p - r;
    aabb->upperBound = transform.p + r;
}

bool b2CircleShape::InscribedSphereAtPoint(const b2Vec2& /*inp*/, const b2Vec2& /*bdp*/, const b2Vec2& /*normal*/, b2Vec2& local_center, b2Scalar &radius) const
{
    if (m_radius < b2_epsilon)
        return false;
    local_center.SetZero();
    radius = m_radius;
    return true;
}

b2Vec2 b2CircleShape::SupportPoint(const b2Vec2& dir) const
{
    return m_radius * dir;
}
