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

#ifndef B2_ELLIPSE_SHAPE_H
#define B2_ELLIPSE_SHAPE_H

#include "b2_api.h"
#include "b2_shape.h"

/// A solid circle shape
class B2_API b2EllipseShape : public b2Shape
{
public:
    b2EllipseShape();

    /// Implement b2Shape.
    b2Shape* Clone(b2BlockAllocator* allocator) const override;

    void SetRadius(b2Scalar r);

    b2Scalar GetRadius() const;

    void SetLocalTransform(const b2Transform& xf) override;

    /// Implement b2Shape.
    bool TestPoint(const b2Transform& transform, const b2Vec2& p) const override;

    /// Implement b2Shape.
    /// @note because the circle is solid, rays that start inside do not hit because the normal is
    /// not defined.
    bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
            const b2Transform& transform) const override;

    /// @see b2Shape::ComputeAABB
    void ComputeAABB(b2AABB* aabb, const b2Transform& transform) const override;

    bool InscribedSphereAtPoint(const b2Vec2& inp, const b2Vec2& bdp, const b2Vec2& normal, b2Vec2& local_center, b2Scalar &radius) const override;

    /// Position
    b2Vec2 m_p;

    /// Radius of a shape. For polygonal shapes this must be b2_polygonRadius. There is no support for
    /// making rounded polygons.
    b2Scalar m_radius;
};

B2_FORCE_INLINE b2EllipseShape::b2EllipseShape()
{
    m_type = e_circle;
    m_radius = b2Scalar(0.0);
    m_p.SetZero();
}

B2_FORCE_INLINE void b2EllipseShape::SetLocalTransform(const b2Transform& xf)
{
    m_p = b2Mul(xf, m_p);
}

B2_FORCE_INLINE void b2EllipseShape::SetRadius(b2Scalar r)
{
    m_radius = r;
}

B2_FORCE_INLINE b2Scalar b2EllipseShape::GetRadius() const
{
    return m_radius;
}
#endif
