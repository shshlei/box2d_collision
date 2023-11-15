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

#include "box2d_collision/b2_shape_capsule.h"

#include "box2d_collision/b2_block_allocator.h"
#include "box2d_collision/b2_math.h"

#include <new>

b2CapsuleShape::b2CapsuleShape()
{
  m_type = e_capsule;
  m_radius = m_lz = m_height = b2Scalar(0.0);
}

b2CapsuleShape::b2CapsuleShape(b2Scalar r, b2Scalar h)
{
  m_type = e_capsule;
  m_radius = r;
  m_lz = b2Scalar(2.0) * h;
  m_height = h;
}

void b2CapsuleShape::SetRadius(b2Scalar r)
{
  m_radius = r;
}

b2Scalar b2CapsuleShape::GetRadius() const
{
  return m_radius;
}

void b2CapsuleShape::SetHeight(b2Scalar h)
{
  m_lz = b2Scalar(2.0) * h;
  m_height = h;
}

b2Scalar b2CapsuleShape::GetHeight() const
{
  return m_height;
}

b2Shape * b2CapsuleShape::Clone(b2BlockAllocator * allocator) const
{
  void * mem = allocator->Allocate(sizeof(b2CapsuleShape));
  b2CapsuleShape * clone = new (mem) b2CapsuleShape;
  *clone = *this;
  return clone;
}

bool b2CapsuleShape::TestPoint(const b2Transform & transform, const b2Vec2 & p) const
{
  b2Vec2 pLocal = b2MulT(transform, p);
  b2Scalar x = b2Abs(pLocal.x()), y = b2Abs(pLocal.y());
  if (x > m_radius) return false;
  if (y <= m_height) return true;
  if (y > m_height + m_radius) return false;
  b2Scalar temp = x * x + (y - m_height) * (y - m_height);
  return temp <= m_radius * m_radius;
}

void b2CapsuleShape::ComputeAABB(b2AABB * aabb, const b2Transform & transform) const
{
  b2Scalar x = b2Abs(transform(0, 1)) * m_height + m_radius;
  b2Scalar y = b2Abs(transform(0, 0)) * m_height + m_radius;
  aabb->min() = transform.translation() - b2Vec2(x, y);
  aabb->max() = transform.translation() + b2Vec2(x, y);
}

bool b2CapsuleShape::InscribedSphereAtPoint(const b2Vec2 & /*inp*/, const b2Vec2 & /*bdp*/, const b2Vec2 & /*normal*/, b2Vec2 & local_center, b2Scalar & radius) const
{
  return false;
}

b2Vec2 b2CapsuleShape::SupportPoint(const b2Vec2 & dir) const
{
  b2Vec2 point = m_radius * dir;
  if (dir.y() >= b2Scalar(0.0))
    point.y() += m_height;
  else
    point.y() -= m_height;
  return point;
}
