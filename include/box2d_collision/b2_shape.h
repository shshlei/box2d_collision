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

#ifndef B2_SHAPE_H
#define B2_SHAPE_H

#include "b2_math.h"

class b2BlockAllocator;

/// A shape is used for collision detection.
class B2_API b2Shape
{
public:

    enum Type
    {
        e_circle = 0,
        e_ellipse = 1,
        e_capsule = 2,
        e_rectangle = 3,
        e_polygon = 4,
        e_typeCount = 5
    };

    virtual ~b2Shape() {}

    /// Get the type of this shape. You can use this to down cast to the concrete shape.
    /// @return the shape type.
    Type GetType() const;

    /// Clone the concrete shape using the provided allocator.
    virtual b2Shape* Clone(b2BlockAllocator* allocator) const = 0;

    /// Test a point for containment in this shape. This only works for convex shapes.
    /// @param xf the shape world transform.
    /// @param p a point in world coordinates.
    virtual bool TestPoint(const b2Transform& xf, const b2Vec2& p) const = 0;

    /// Given a transform, compute the associated axis aligned bounding box for a child shape.
    /// @param aabb returns the axis aligned box.
    /// @param xf the world transform of the shape.
    virtual void ComputeAABB(b2AABB* aabb, const b2Transform& xf) const = 0;

    /// Given an interior point and a boundary point of the shape, find an inscribed sphere. This only works for convex shapes.
    /// @param inp an interior point in local coordinates.
    /// @param bdp a boundary point in local coordinates.
    /// @param normal a normal vector from inp to bdp.
    /// @param local_center the inscribed sphere's center.
    /// @param radius the inscribed sphere's radius.
    virtual bool InscribedSphereAtPoint(const b2Vec2& inp, const b2Vec2& bdp, const b2Vec2& normal, b2Vec2& local_center, b2Scalar &radius) const = 0;

    /// Given a dir, compute the support mapping point.
    /// @param dir a dir in local coordinates.
    /// @param point the support mapping point in local coordinates.
    virtual b2Vec2 SupportPoint(const b2Vec2& dir) const = 0;

    Type m_type;
};

B2_FORCE_INLINE b2Shape::Type b2Shape::GetType() const
{
    return m_type;
}

#endif
