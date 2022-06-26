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

#include "box2d_collision/b2_collision.h"
#include "box2d_collision/b2_circle_shape.h"
#include "box2d_collision/b2_polygon_shape.h"

bool b2CollideCircles(b2Manifold* manifold,
        const b2CircleShape* circleA, const b2Transform& xfA,
        const b2CircleShape* circleB, const b2Transform& xfB)
{
    b2Vec2 pA = b2Mul(xfA, circleA->m_p);
    b2Vec2 pB = b2Mul(xfB, circleB->m_p);

    b2Vec2 d = pB - pA;
    b2Scalar distSqr = b2Dot(d, d);
    b2Scalar rA = circleA->m_radius, rB = circleB->m_radius;
    b2Scalar radius = rA + rB;
    if (distSqr > radius * radius)
        return false;
    if (manifold)
    {
        manifold->type = b2Manifold::e_circles;
        manifold->localPoint = circleA->m_p;
        manifold->localNormal.SetZero();
        manifold->pointCount = 1;
        manifold->separations[0] = b2Sqrt(distSqr) - radius;
        manifold->points[0].localPoint = circleB->m_p;
        manifold->points[0].id.key = 0;
    }
    return true;
}

bool b2CollidePolygonAndCircle(b2Manifold* manifold,
        const b2PolygonShape* polygonA, const b2Transform& xfA,
        const b2CircleShape* circleB, const b2Transform& xfB)
{
    // Compute circle position in the frame of the polygon.
    b2Vec2 c = b2Mul(xfB, circleB->m_p);
    b2Vec2 cLocal = b2MulT(xfA, c);

    // Find the min separating edge.
    int32 normalIndex = 0;
    b2Scalar separation = -b2_maxFloat;
    b2Scalar radius = polygonA->m_radius + circleB->m_radius;
    int32 vertexCount = polygonA->m_count;
    const b2Vec2* vertices = polygonA->m_vertices;
    const b2Vec2* normals = polygonA->m_normals;

    for (int32 i = 0; i < vertexCount; ++i)
    {
        b2Scalar s = b2Dot(normals[i], cLocal - vertices[i]);
        if (s > radius)
            // Early out.
            return false;
        if (s > separation)
        {
            separation = s;
            normalIndex = i;
        }
    }

    // If the center is inside the polygon ...
    if (separation < b2_epsilon)
    {
        if (manifold)
        {
            manifold->separations[0] = separation - circleB->m_radius;
            manifold->pointCount = 1;
            manifold->type = b2Manifold::e_faceA;
            manifold->localNormal = normals[normalIndex];
            manifold->localPoint = cLocal - manifold->localNormal * separation;
            manifold->points[0].localPoint = circleB->m_p;
            manifold->points[0].id.key = 0;
        }
        return true;
    }

    // Vertices that subtend the incident face.
    int32 vertIndex1 = normalIndex;
    int32 vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
    b2Vec2 v1 = vertices[vertIndex1];
    b2Vec2 v2 = vertices[vertIndex2];
    // Compute barycentric coordinates
    b2Scalar u1 = b2Dot(cLocal - v1, v2 - v1);
    b2Scalar u2 = b2Dot(cLocal - v2, v1 - v2);
    if (u1 <= b2Scalar(0.0))
    {
        double d = b2DistanceSquared(cLocal, v1);
        if (d > radius * radius)
            return false;
        if (manifold)
        {
            manifold->separations[0] = b2Sqrt(d) - circleB->m_radius;
            manifold->pointCount = 1;
            manifold->type = b2Manifold::e_faceA;
            manifold->localNormal = cLocal - v1;
            manifold->localNormal.Normalize();
            manifold->localPoint = v1;
            manifold->points[0].localPoint = circleB->m_p;
            manifold->points[0].id.key = 0;
        }
    }
    else if (u2 <= b2Scalar(0.0))
    {
        double d = b2DistanceSquared(cLocal, v2);
        if (d > radius * radius)
            return false;
        if (manifold)
        {
            manifold->separations[0] = b2Sqrt(d) - circleB->m_radius;
            manifold->pointCount = 1;
            manifold->type = b2Manifold::e_faceA;
            manifold->localNormal = cLocal - v2;
            manifold->localNormal.Normalize();
            manifold->localPoint = v2;
            manifold->points[0].localPoint = circleB->m_p;
            manifold->points[0].id.key = 0;
        }
    }
    else
    {
        b2Vec2 faceCenter = b2Scalar(0.5) * (v1 + v2);
        b2Scalar s = b2Dot(cLocal - faceCenter, normals[vertIndex1]);
        if (s > radius)
            return false;
        if (manifold)
        {
            manifold->separations[0] = separation - circleB->m_radius;
            manifold->pointCount = 1;
            manifold->type = b2Manifold::e_faceA;
            manifold->localNormal = normals[vertIndex1];
            manifold->localPoint = cLocal - manifold->localNormal * separation;
            manifold->points[0].localPoint = circleB->m_p;
            manifold->points[0].id.key = 0;
        }
    }
    return true;
}
