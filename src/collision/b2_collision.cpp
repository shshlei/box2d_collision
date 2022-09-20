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

#include "box2d_collision/b2_shape.h"
#include "box2d_collision/b2_collision.h"
#include "box2d_collision/b2_distance.h"

void b2InscribedSpheres::Initialize(const b2ContactResult* manifold, const b2Shape *shape1, const b2Shape *shape2)
{
    b2Vec2 normal = manifold->normal;
    b2Vec2 p1 = manifold->points[0];
    b2Vec2 p2 = manifold->points[1];
    const b2Transform& xf1 = manifold->transforms[0];
    const b2Transform& xf2 = manifold->transforms[1];
    if (has_sphere1 = shape1->InscribedSphereAtPoint(b2MulT(xf1.q, p2 - xf1.p), manifold->local_points[0], b2MulT(xf1.q, normal), local_center1, radius1))
        center1 = xf1.p + b2Mul(xf1.q, local_center1);
    if (has_sphere2 = shape2->InscribedSphereAtPoint(b2MulT(xf2.q, p1 - xf2.p), manifold->local_points[1], b2MulT(xf2.q, -normal), local_center2, radius2))
        center2 = xf2.p + b2Mul(xf2.q, local_center2);
}

void b2InscribedSpheres::Initialize(const b2Manifold* manifold, const b2Shape *shape1, const b2Transform& xf1, const b2Shape *shape2, const b2Transform& xf2)
{
    b2Vec2 normal = manifold->normal;
    b2Vec2 p1 = manifold->point - manifold->separation * normal;
    b2Vec2 p2 = manifold->point;
    if (has_sphere1 = shape1->InscribedSphereAtPoint(b2MulT(xf1.q, p2 - xf1.p), b2MulT(xf1.q, p1 - xf1.p), b2MulT(xf1.q, normal), local_center1, radius1))
        center1 = xf1.p + b2Mul(xf1.q, local_center1);
    if (has_sphere2 = shape2->InscribedSphereAtPoint(b2MulT(xf2.q, p1 - xf2.p), b2MulT(xf2.q, p2 - xf2.p), b2MulT(xf2.q, -normal), local_center2, radius2))
        center2 = xf2.p + b2Mul(xf2.q, local_center2);
}

// From Real-time Collision Detection, p179.
bool b2AABB::RayCast(b2RayCastOutput* output, const b2RayCastInput& input) const
{
    b2Scalar tmin = -b2_maxFloat;
    b2Scalar tmax = b2_maxFloat;

    b2Vec2 p = input.p1;
    b2Vec2 d = input.p2 - input.p1;
    b2Vec2 absD = b2Abs(d);

    b2Vec2 normal;

    for (int32 i = 0; i < 2; ++i)
    {
        if (absD(i) < b2_epsilon)
        {
            // Parallel.
            if (p(i) < lowerBound(i) || upperBound(i) < p(i))
            {
                return false;
            }
        }
        else
        {
            b2Scalar inv_d = b2Scalar(1.0) / d(i);
            b2Scalar t1 = (lowerBound(i) - p(i)) * inv_d;
            b2Scalar t2 = (upperBound(i) - p(i)) * inv_d;

            // Sign of the normal vector.
            b2Scalar s = -b2Scalar(1.0);

            if (t1 > t2)
            {
                b2Swap(t1, t2);
                s = b2Scalar(1.0);
            }

            // Push the min up
            if (t1 > tmin)
            {
                normal.SetZero();
                normal(i) = s;
                tmin = t1;
            }

            // Pull the max down
            tmax = b2Min(tmax, t2);

            if (tmin > tmax)
            {
                return false;
            }
        }
    }

    // Does the ray start inside the box?
    // Does the ray intersect beyond the max fraction?
    if (tmin < b2Scalar(0.0) || input.maxFraction < tmin)
    {
        return false;
    }

    // Intersection.
    output->fraction = tmin;
    output->normal = normal;
    return true;
}

bool b2TestOverlap(const b2Shape* shapeA, const b2Shape* shapeB, const b2Transform& xfA, const b2Transform& xfB)
{
    b2DistanceInput input;
    input.proxyA.Set(shapeA);
    input.proxyB.Set(shapeB);
    input.transformA = xfA;
    input.transformB = xfB;
    input.useRadii = true;

    b2SimplexCache cache;
    cache.count = 0;

    b2DistanceOutput output;

    b2Distance(&output, &cache, &input);

    return output.distance < b2Scalar(10.0) * b2_epsilon;
}
