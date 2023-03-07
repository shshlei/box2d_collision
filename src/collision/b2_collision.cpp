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
#include "box2d_collision/b2_distance.h"

bool b2CollideShapes(b2Manifold* manifold,
        const b2Shape* shapeA, const b2Transform& xfA,
        const b2Shape* shapeB, const b2Transform& xfB, bool separationStop)
{
    if (shapeA->GetType() == b2Shape::e_circle && shapeB->GetType() == b2Shape::e_circle)
        return b2CollideCircles(manifold, (const b2CircleShape *)shapeA, xfA, (const b2CircleShape *)shapeB, xfB, separationStop);
    else if (shapeA->GetType() == b2Shape::e_capsule && shapeB->GetType() == b2Shape::e_capsule)
        return b2CollideCapsules(manifold, (const b2CapsuleShape *)shapeA, xfA, (const b2CapsuleShape *)shapeB, xfB, separationStop);
    else if (shapeA->GetType() == b2Shape::e_polygon && shapeB->GetType() == b2Shape::e_polygon)
        return b2CollidePolygons(manifold, (const b2PolygonShape *)shapeA, xfA, (const b2PolygonShape *)shapeB, xfB, separationStop);
    else if (shapeA->GetType() == b2Shape::e_circle)
    {
        bool collision = false;
        if (shapeB->GetType() == b2Shape::e_capsule)
            collision = b2CollideCapsuleCircle(manifold, (const b2CapsuleShape *)shapeB, xfB, (const b2CircleShape *)shapeA, xfA, separationStop);
        else if (shapeB->GetType() == b2Shape::e_rectangle)
            collision = b2CollideRectangleCircle(manifold, (const b2RectangleShape *)shapeB, xfB, (const b2CircleShape *)shapeA, xfA, separationStop);
        else if (shapeB->GetType() == b2Shape::e_polygon)
            collision = b2CollidePolygonCircle(manifold, (const b2PolygonShape *)shapeB, xfB, (const b2CircleShape *)shapeA, xfA, separationStop);
        else 
        {
            b2ShapeDistance dist;
            if (manifold)
            {
                b2Scalar d;
                b2Vec2 p1, p2;
                collision = !dist.Distance(shapeA, xfA, shapeB, xfB, &d, &p1, &p2);
                if (!collision)
                {
                    manifold->separation = d;
                    manifold->point = p2;
                    manifold->normal = (p2 - p1).Normalized();
                }
                else 
                    manifold->separation = b2Scalar(0.0);
            }
            else 
                collision = !dist.Separation(shapeA, xfA, shapeB, xfB);
            return collision;
        }
        if (manifold)
        {
            manifold->normal = -manifold->normal;
            manifold->point = manifold->point + manifold->normal * manifold->separation;
        }
        return collision;
    }
    else if (shapeB->GetType() == b2Shape::e_circle)
    {
        if (shapeA->GetType() == b2Shape::e_capsule)
            return b2CollideCapsuleCircle(manifold, (const b2CapsuleShape *)shapeA, xfA, (const b2CircleShape *)shapeB, xfB, separationStop);
        else if (shapeA->GetType() == b2Shape::e_rectangle)       
            return b2CollideRectangleCircle(manifold, (const b2RectangleShape *)shapeA, xfA, (const b2CircleShape *)shapeB, xfB, separationStop);
        else if (shapeA->GetType() == b2Shape::e_polygon)       
            return b2CollidePolygonCircle(manifold, (const b2PolygonShape *)shapeA, xfA, (const b2CircleShape *)shapeB, xfB, separationStop);
        else 
        {
            bool collision = false;
            b2ShapeDistance dist;
            if (manifold)
            {
                b2Scalar d;
                b2Vec2 p1, p2;
                collision = !dist.Distance(shapeA, xfA, shapeB, xfB, &d, &p1, &p2);
                if (!collision)
                {
                    manifold->separation = d;
                    manifold->point = p2;
                    manifold->normal = (p2 - p1).Normalized();
                }
                else 
                    manifold->separation = b2Scalar(0.0);
            }
            else 
                collision = !dist.Separation(shapeA, xfA, shapeB, xfB);
            return collision;
        }
    }
    else 
    {
        bool collision = false;
        b2ShapeDistance dist;
        if (manifold)
        {
            b2Scalar d;
            b2Vec2 p1, p2;
            collision = !dist.Distance(shapeA, xfA, shapeB, xfB, &d, &p1, &p2);
            if (!collision)
            {
                manifold->separation = d;
                manifold->point = p2;
                manifold->normal = (p2 - p1).Normalized();
            }
            else 
                manifold->separation = b2Scalar(0.0);
        }
        else 
            collision = !dist.Separation(shapeA, xfA, shapeB, xfB);
        return collision;
    }
}

bool b2CollideShapes(const b2Shape* shapeA, const b2Transform& xfA, const b2Shape* shapeB, const b2Transform& xfB)
{
    return b2CollideShapes(nullptr, shapeA, xfA, shapeB, xfB);
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
    if (has_sphere1 && has_sphere2)
    {
        if (radius1 >= b2Scalar(5.0) * radius2)
            has_sphere2 = false;
        if (radius2 >= b2Scalar(5.0) * radius1)
            has_sphere1 = false;
    }
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
    if (has_sphere1 && has_sphere2)
    {
        if (radius1 >= b2Scalar(5.0) * radius2)
            has_sphere2 = false;
        if (radius2 >= b2Scalar(5.0) * radius1)
            has_sphere1 = false;
    }
}

/*
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
*/
