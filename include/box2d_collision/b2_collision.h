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

#ifndef B2_COLLISION_H
#define B2_COLLISION_H

#include <limits.h>
#include <string>

#include "b2_api.h"
#include "b2_math.h"

/// @file
/// Structures and functions used for computing contact points, distance
/// queries, and TOI queries.

class b2Shape;
class b2CircleShape;
class b2PolygonShape;
class b2EllipseShape;

/// A manifold for two touching convex shapes.
struct B2_API b2Manifold
{
    b2Vec2 normal;
    b2Vec2 point;
    b2Scalar separation;
};

struct B2_API b2ContactResult
{
    std::string names[2];
    uint32 shape_id[2];
    b2Vec2 normal;
    b2Vec2 points[2];
    b2Vec2 local_points[2];
    b2Transform transforms[2];
    b2Scalar separation;
};

/// This is used to compute the current state of a contact manifold.
struct B2_API b2InscribedSpheres
{
    void Initialize(const b2ContactResult* manifold, const b2Shape *shape1, const b2Shape *shape2);
    void Initialize(const b2Manifold* manifold, const b2Shape *shape1, const b2Transform& xf1, const b2Shape *shape2, const b2Transform& xf2);
    bool has_sphere1, has_sphere2;
    b2Vec2 local_center1, local_center2;
    b2Vec2 center1, center2;
    b2Scalar radius1, radius2;
};

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
struct B2_API b2RayCastInput
{
    b2Vec2 p1, p2;
    b2Scalar maxFraction;
};

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
struct B2_API b2RayCastOutput
{
    b2Vec2 normal;
    b2Scalar fraction;
};

/// Compute the collision manifold between two circles.
B2_API bool b2CollideCircles(b2Manifold* manifold,
        const b2CircleShape* circleA, const b2Transform& xfA,
        const b2CircleShape* circleB, const b2Transform& xfB, bool separationStop = true);

/// Compute the collision manifold between a polygon and a circle.
B2_API bool b2CollidePolygonAndCircle(b2Manifold* manifold,
        const b2PolygonShape* polygonA, const b2Transform& xfA,
        const b2CircleShape* circleB, const b2Transform& xfB, bool separationStop = true);

/// Compute the collision manifold between two polygons.
B2_API bool b2CollidePolygons(b2Manifold* manifold,
        const b2PolygonShape* polygonA, const b2Transform& xfA,
        const b2PolygonShape* polygonB, const b2Transform& xfB, bool separationStop = true);

/// Determine if two generic shapes overlap.
B2_API bool b2TestOverlap(const b2Shape* shapeA, const b2Shape* shapeB, const b2Transform& xfA, const b2Transform& xfB);

/// An axis aligned bounding box.
struct B2_API b2AABB
{
    /// Verify that the bounds are sorted.
    bool IsValid() const;

    B2_FORCE_INLINE void SetMin(b2Scalar min)
    {
        lowerBound.Set(min, min);
    }

    B2_FORCE_INLINE void SetMin(b2Scalar minx, b2Scalar miny)
    {
        lowerBound.Set(minx, miny);
    }

    B2_FORCE_INLINE void SetMax(b2Scalar max)
    {
        upperBound.Set(max, max);
    }

    B2_FORCE_INLINE void SetMax(b2Scalar maxx, b2Scalar maxy)
    {
        upperBound.Set(maxx, maxy);
    }

    /// Get the center of the AABB.
    B2_FORCE_INLINE b2Vec2 GetCenter() const
    {
        return b2Scalar(0.5) * (lowerBound + upperBound);
    }

    /// Get the extents of the AABB (half-widths).
    B2_FORCE_INLINE b2Vec2 GetExtents() const
    {
        return b2Scalar(0.5) * (upperBound - lowerBound);
    }

    /// Get the perimeter length
    B2_FORCE_INLINE b2Scalar GetPerimeter() const
    {
        b2Scalar wx = upperBound.x - lowerBound.x;
        b2Scalar wy = upperBound.y - lowerBound.y;
        return b2Scalar(2.0) * (wx + wy);
    }

    /// Combine an AABB into this one.
    B2_FORCE_INLINE void Combine(const b2AABB& aabb)
    {
        lowerBound = b2Min(lowerBound, aabb.lowerBound);
        upperBound = b2Max(upperBound, aabb.upperBound);
    }

    /// Combine two AABBs into this one.
    B2_FORCE_INLINE void Combine(const b2AABB& aabb1, const b2AABB& aabb2)
    {
        lowerBound = b2Min(aabb1.lowerBound, aabb2.lowerBound);
        upperBound = b2Max(aabb1.upperBound, aabb2.upperBound);
    }

    /// Does this aabb contain the provided AABB.
    B2_FORCE_INLINE bool Contains(const b2AABB& aabb) const
    {
        bool result = true;
        result = result && lowerBound.x <= aabb.lowerBound.x;
        result = result && lowerBound.y <= aabb.lowerBound.y;
        result = result && aabb.upperBound.x <= upperBound.x;
        result = result && aabb.upperBound.y <= upperBound.y;
        return result;
    }

    /// Does this aabb contain the provided point.
    B2_FORCE_INLINE bool Contains(const b2Vec2& aabb) const
    {
        if (aabb.x < lowerBound.x)
            return false;
        if (aabb.x > upperBound.x)
            return false;
        if (aabb.y < lowerBound.y)
            return false;
        if (aabb.y > upperBound.y)
            return false;
        return true;
    }

    bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input) const;

    b2Vec2 lowerBound;	///< the lower vertex
    b2Vec2 upperBound;	///< the upper vertex
};

// ---------------- Inline Functions ------------------------------------------

B2_FORCE_INLINE bool b2AABB::IsValid() const
{
    b2Vec2 d = upperBound - lowerBound;
    bool valid = d.x >= b2Scalar(0.0) && d.y >= b2Scalar(0.0);
    valid = valid && lowerBound.IsValid() && upperBound.IsValid();
    return valid;
}

B2_FORCE_INLINE bool b2TestOverlap(const b2AABB& a, const b2AABB& b)
{
    b2Vec2 d1, d2;
    d1 = b.lowerBound - a.upperBound;
    d2 = a.lowerBound - b.upperBound;

    if (d1.x > b2Scalar(0.0) || d1.y > b2Scalar(0.0))
        return false;
    if (d2.x > b2Scalar(0.0) || d2.y > b2Scalar(0.0))
        return false;
    return true;
}

#endif
