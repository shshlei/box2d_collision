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

#include "box2d_collision/b2_circle_shape.h"
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
    b2Vec2 center = transform.p + b2Mul(transform.q, m_p);
    b2Vec2 d = p - center;
    return b2Dot(d, d) <= m_radius * m_radius;
}

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius
bool b2EllipseShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
        const b2Transform& transform) const
{
    b2Vec2 position = transform.p + b2Mul(transform.q, m_p);
    b2Vec2 s = input.p1 - position;
    b2Scalar b = b2Dot(s, s) - m_radius * m_radius;

    // Solve quadratic equation.
    b2Vec2 r = input.p2 - input.p1;
    b2Scalar c =  b2Dot(s, r);
    b2Scalar rr = b2Dot(r, r);
    b2Scalar sigma = c * c - rr * b;

    // Check for negative discriminant and short segment.
    if (sigma < b2Scalar(0.0) || rr < b2_epsilon)
    {
        return false;
    }

    // Find the point of intersection of the line with the circle.
    b2Scalar a = -(c + b2Sqrt(sigma));

    // Is the intersection point on the segment?
    if (b2Scalar(0.0) <= a && a <= input.maxFraction * rr)
    {
        a /= rr;
        output->fraction = a;
        output->normal = s + a * r;
        output->normal.Normalize();
        return true;
    }

    return false;
}

void b2EllipseShape::ComputeAABB(b2AABB* aabb, const b2Transform& transform) const
{
    b2Vec2 p = transform.p + b2Mul(transform.q, m_p);
    aabb->lowerBound.Set(p.x - m_radius, p.y - m_radius);
    aabb->upperBound.Set(p.x + m_radius, p.y + m_radius);
}

bool b2EllipseShape::InscribedSphereAtPoint(const b2Vec2& /*inp*/, const b2Vec2& /*bdp*/, const b2Vec2& /*normal*/, b2Vec2& local_center, b2Scalar &radius) const
{
    if (m_radius < B2_EPSILON)
        return false;
    local_center = m_p;
    radius = m_radius;
    return true;
}
