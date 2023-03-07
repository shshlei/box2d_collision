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

#ifndef B2_CAPSULE_SHAPE_H
#define B2_CAPSULE_SHAPE_H

#include "b2_shape.h"

/// A solid capsule shape
class B2_API b2CapsuleShape : public b2Shape
{
public:
    b2CapsuleShape();

    b2CapsuleShape(b2Scalar r, b2Scalar h);

    void SetRadius(b2Scalar r);

    b2Scalar GetRadius() const;

    void SetHeight(b2Scalar h);

    b2Scalar GetHeight() const;

    /// Implement b2Shape.
    b2Shape* Clone(b2BlockAllocator* allocator) const override;

    /// Implement b2Shape.
    bool TestPoint(const b2Transform& transform, const b2Vec2& p) const override;

    /// @see b2Shape::ComputeAABB
    void ComputeAABB(b2AABB* aabb, const b2Transform& transform) const override;

    bool InscribedSphereAtPoint(const b2Vec2& inp, const b2Vec2& bdp, const b2Vec2& normal, b2Vec2& local_center, b2Scalar &radius) const override;

    b2Vec2 SupportPoint(const b2Vec2& dir) const override;

private:
    b2Scalar m_radius, m_lz, m_height;
};

B2_FORCE_INLINE b2CapsuleShape::b2CapsuleShape()
{
    m_type = e_capsule;
    m_radius = m_lz = m_height = b2Scalar(0.0);
}

B2_FORCE_INLINE b2CapsuleShape::b2CapsuleShape(b2Scalar r, b2Scalar h)
{
    m_type = e_capsule;
    m_radius = r;
    m_lz = b2Scalar(2.0) * h;
    m_height = h;
}

B2_FORCE_INLINE void b2CapsuleShape::SetRadius(b2Scalar r)
{
    m_radius = r;
}

B2_FORCE_INLINE b2Scalar b2CapsuleShape::GetRadius() const
{
    return m_radius;
}

B2_FORCE_INLINE void b2CapsuleShape::SetHeight(b2Scalar h)
{
    m_lz = b2Scalar(2.0) * h;
    m_height = h;
}

B2_FORCE_INLINE b2Scalar b2CapsuleShape::GetHeight() const
{
    return m_height;
}

#endif
