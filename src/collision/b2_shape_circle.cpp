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
#include "box2d_collision/b2_math.h"

#include <new>

b2Shape::Type b2Shape::GetType() const
{
  return m_type;
}

b2CircleShape::b2CircleShape()
{
  m_type = e_circle;
  m_radius = b2Scalar(0.0);
}

b2CircleShape::b2CircleShape(b2Scalar r)
{
  m_type = e_circle;
  m_radius = r;
}

void b2CircleShape::SetRadius(b2Scalar r)
{
  m_radius = r;
}

b2Scalar b2CircleShape::GetRadius() const
{
  return m_radius;
}

b2Shape * b2CircleShape::Clone(b2BlockAllocator * allocator) const
{
  void * mem = allocator->Allocate(sizeof(b2CircleShape));
  b2CircleShape * clone = new (mem) b2CircleShape;
  *clone = *this;
  return clone;
}

bool b2CircleShape::TestPoint(const b2Transform & transform, const b2Vec2 & p) const
{
  b2Vec2 d = p - transform.translation();
  return b2Dot(d, d) <= m_radius * m_radius;
}

void b2CircleShape::ComputeAABB(b2AABB * aabb, const b2Transform & transform) const
{
  b2Vec2 r(m_radius, m_radius);
  aabb->min() = transform.translation() - r;
  aabb->max() = transform.translation() + r;
}

bool b2CircleShape::InscribedSphereAtPoint(const b2Vec2 & /*inp*/, const b2Vec2 & /*bdp*/, const b2Vec2 & /*normal*/, b2Vec2 & local_center, b2Scalar & radius) const
{
  if (m_radius < B2_EPSILON) return false;
  local_center.setZero();
  radius = m_radius;
  return true;
}

b2Vec2 b2CircleShape::SupportPoint(const b2Vec2 & dir) const
{
  return m_radius * dir;
}
