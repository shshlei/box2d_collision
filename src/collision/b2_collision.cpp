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

void b2WorldManifold::Initialize(const b2Manifold* manifold,
        const b2Transform& xfA, b2Scalar radiusA,
        const b2Transform& xfB, b2Scalar radiusB)
{
    if (manifold->pointCount == 0)
        return;
    pointCount = manifold->pointCount;
    switch (manifold->type)
    {
        case b2Manifold::e_circles:
            {
                normal.Set(b2Scalar(1.0), b2Scalar(0.0));
                b2Vec2 pointA = b2Mul(xfA, manifold->localPoint);
                b2Vec2 pointB = b2Mul(xfB, manifold->points[0].localPoint);
                b2Scalar d = b2DistanceSquared(pointA, pointB);
                if (d > b2_epsilon * b2_epsilon)
                {
                    normal = pointB - pointA;
                    normal.Normalize();
                }
                separations[0] = -manifold->separations[0];
                points[0] = pointB - radiusB * normal;
            }
            break;

        case b2Manifold::e_faceA:
            {
                normal = b2Mul(xfA.q, manifold->localNormal);
                for (int32 i = 0; i < manifold->pointCount; ++i)
                {
                    b2Vec2 clipPoint = b2Mul(xfB, manifold->points[i].localPoint);
                    points[i] = clipPoint - radiusB * normal;
                    separations[i] = -manifold->separations[i];
                }
            }
            break;

        case b2Manifold::e_faceB:
            {
                normal = b2Mul(xfB.q, manifold->localNormal);
                for (int32 i = 0; i < manifold->pointCount; ++i)
                {
                    b2Vec2 clipPoint = b2Mul(xfA, manifold->points[i].localPoint);
                    points[i] = clipPoint + (radiusB - manifold->separations[i]) * normal;
                    separations[i] = -manifold->separations[i];
                }
                // Ensure normal points from A to B.
                normal = -normal;
            }
            break;
    }
}

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

void b2InscribedSpheres::Initialize(const b2WorldManifold* manifold, const b2Shape *shape1, const b2Transform& xf1, const b2Shape *shape2, const b2Transform& xf2)
{
    int32 bestIndex = 0;
    for (int32 i = 1; i < manifold->pointCount; i++)
    {
        if (manifold->separations[i] > manifold->separations[bestIndex])
            bestIndex = i;
    }
    b2Vec2 normal = manifold->normal;
    b2Vec2 p1 = manifold->points[bestIndex] + manifold->separations[bestIndex] * normal;
    b2Vec2 p2 = manifold->points[bestIndex];
    if (has_sphere1 = shape1->InscribedSphereAtPoint(b2MulT(xf1.q, p2 - xf1.p), b2MulT(xf1.q, p1 - xf1.p), b2MulT(xf1.q, normal), local_center1, radius1))
        center1 = xf1.p + b2Mul(xf1.q, local_center1);
    if (has_sphere2 = shape2->InscribedSphereAtPoint(b2MulT(xf2.q, p1 - xf2.p), b2MulT(xf2.q, p2 - xf2.p), b2MulT(xf2.q, -normal), local_center2, radius2))
        center2 = xf2.p + b2Mul(xf2.q, local_center2);
}

void b2GetPointStates(b2PointState state1[b2_maxManifoldPoints], b2PointState state2[b2_maxManifoldPoints],
        const b2Manifold* manifold1, const b2Manifold* manifold2)
{
    for (int32 i = 0; i < b2_maxManifoldPoints; ++i)
    {
        state1[i] = b2_nullState;
        state2[i] = b2_nullState;
    }

    // Detect persists and removes.
    for (int32 i = 0; i < manifold1->pointCount; ++i)
    {
        b2ContactID id = manifold1->points[i].id;

        state1[i] = b2_removeState;

        for (int32 j = 0; j < manifold2->pointCount; ++j)
        {
            if (manifold2->points[j].id.key == id.key)
            {
                state1[i] = b2_persistState;
                break;
            }
        }
    }

    // Detect persists and adds.
    for (int32 i = 0; i < manifold2->pointCount; ++i)
    {
        b2ContactID id = manifold2->points[i].id;

        state2[i] = b2_addState;

        for (int32 j = 0; j < manifold1->pointCount; ++j)
        {
            if (manifold1->points[j].id.key == id.key)
            {
                state2[i] = b2_persistState;
                break;
            }
        }
    }
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

// Sutherland-Hodgman clipping.
int32 b2ClipSegmentToLine(b2ClipVertex vOut[2], const b2ClipVertex vIn[2],
        const b2Vec2& normal, b2Scalar offset, int32 vertexIndexA)
{
    // Start with no output points
    int32 count = 0;

    // Calculate the distance of end points to the line
    b2Scalar distance0 = b2Dot(normal, vIn[0].v) - offset;
    b2Scalar distance1 = b2Dot(normal, vIn[1].v) - offset;

    // If the points are behind the plane
    if (distance0 <= b2Scalar(0.0)) vOut[count++] = vIn[0];
    if (distance1 <= b2Scalar(0.0)) vOut[count++] = vIn[1];

    // If the points are on different sides of the plane
    if (distance0 * distance1 < b2Scalar(0.0))
    {
        // Find intersection point of edge and plane
        b2Scalar interp = distance0 / (distance0 - distance1);
        vOut[count].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);

        // VertexA is hitting edgeB.
        vOut[count].id.cf.indexA = static_cast<uint8>(vertexIndexA);
        vOut[count].id.cf.indexB = vIn[0].id.cf.indexB;
        vOut[count].id.cf.typeA = b2ContactFeature::e_vertex;
        vOut[count].id.cf.typeB = b2ContactFeature::e_face;
        ++count;

//        b2Assert(count == 2);
    }

    return count;
}

bool b2TestOverlap(const b2Shape* shapeA, int32 indexA,
        const b2Shape* shapeB, int32 indexB,
        const b2Transform& xfA, const b2Transform& xfB)
{
    b2DistanceInput input;
    input.proxyA.Set(shapeA, indexA);
    input.proxyB.Set(shapeB, indexB);
    input.transformA = xfA;
    input.transformB = xfB;
    input.useRadii = true;

    b2SimplexCache cache;
    cache.count = 0;

    b2DistanceOutput output;

    b2Distance(&output, &cache, &input);

    return output.distance < b2Scalar(10.0) * b2_epsilon;
}
