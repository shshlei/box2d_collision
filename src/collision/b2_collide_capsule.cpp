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

/// In capsule's local frame
static b2Vec2 b2CapsulePointDir(b2Scalar height, const b2Vec2& point, b2Scalar &distSqr)
{
    b2Vec2 dir(point.x, b2Scalar(0.0));
    distSqr = point.x * point.x;
    if (point.y > height)
    {
        dir.y = point.y - height;
        distSqr += dir.y * dir.y;
    }
    else if (point.y < -height)
    {
        dir.y = point.y + height;
        distSqr += dir.y * dir.y;
    }
    return dir;
}

bool b2CollideCapsuleCircle(b2Manifold* manifold,
        const b2CapsuleShape* capsuleA, const b2Transform& xfA,
        const b2CircleShape* circleB, const b2Transform& xfB, bool separationStop)
{
    const b2Vec2& c = xfB.p;
    b2Vec2 cLocal = b2MulT(xfA, c);

    b2Scalar height = capsuleA->GetHeight();
    b2Scalar distSqr;
    b2Vec2 d = b2CapsulePointDir(height, cLocal, distSqr);

    b2Scalar radius = capsuleA->GetRadius() + circleB->m_radius;
    bool collision = distSqr <= radius * radius;
    if (!collision && separationStop)
        return collision;
    if (manifold)
    {
        manifold->normal.Set(b2Scalar(1.0), b2Scalar(0.0));
        b2Scalar l = b2Sqrt(distSqr);
        if (l > b2_epsilon)
            manifold->normal = d / l;
        manifold->normal = b2Mul(xfA.q, manifold->normal);
        manifold->separation = l - radius;
        manifold->point = c - circleB->m_radius * manifold->normal;
    }
    return collision;
}

/// Closest point of capsule2 in relative to capsule1
static bool b2FindClosestPoint(const b2CapsuleShape* capsule1, const b2Transform& xf1,
        const b2CapsuleShape* capsule2, const b2Transform& xf2,
        b2Vec2 &point, b2Vec2 &dir, b2Scalar &distSqr)
{
    /// The two segment points of capsule2 
    b2Scalar height = capsule2->GetHeight();
    b2Vec2 half_arm(-xf2.q.s, xf2.q.c);
    b2Vec2 a2(xf2.p - height * half_arm);
    b2Vec2 b2(xf2.p + height * half_arm);

    /// Local points in capsule1' frame
    b2Vec2 a = b2MulT(xf1, a2);
    b2Vec2 b = b2MulT(xf1, b2);

    /// Normal direction 
    height = capsule1->GetHeight();   
    b2Scalar da, db;
    b2Vec2 dira = b2CapsulePointDir(height, a, da); 
    b2Vec2 dirb = b2CapsulePointDir(height, b, db);

    if (da <= db)
    {
        point = a;
        dir = dira;
        distSqr = da;
    }
    else
    {
        point = b;
        dir = dirb;
        distSqr = db;
    }
    return a.x * b.x >= b2Scalar(0.0); // not cross
}

bool b2CollideCapsules(b2Manifold* manifold,
        const b2CapsuleShape* capsuleA, const b2Transform& xfA,
        const b2CapsuleShape* capsuleB, const b2Transform& xfB, bool separationStop)
{
    b2Vec2 pointb, dirb, pointa, dira;
    b2Scalar distSqrb, distSqra;

    bool notcrossa = b2FindClosestPoint(capsuleA, xfA, capsuleB, xfB, pointb, dirb, distSqrb);
    bool notcrossb = b2FindClosestPoint(capsuleB, xfB, capsuleA, xfA, pointa, dira, distSqra);

    bool flip = false, cross = false;
    const b2CapsuleShape* capsule1 = capsuleA;
    const b2CapsuleShape* capsule2 = capsuleB;
    b2Transform xf1 = xfA;
    b2Vec2 point = pointb, dir = dirb;
    b2Scalar distSqr = distSqrb;

    if (notcrossa == notcrossb)
    {
        cross = !notcrossa;
        if (distSqra < distSqrb)
        {
            flip = true;

            capsule1 = capsuleB;
            capsule2 = capsuleA;
            xf1 = xfB;

            point = pointa;
            dir = dira;
            distSqr = distSqra;
        }
    }
    else if (notcrossb)
    {
        flip = true;

        capsule1 = capsuleB;
        capsule2 = capsuleA;
        xf1 = xfB;

        point = pointa;
        dir = dira;
        distSqr = distSqra;       
    }

    b2Scalar rA = capsule1->GetRadius(), rB = capsule2->GetRadius();
    b2Scalar radius = rA + rB;
    bool collision = true;
    if (cross)
    {
        if (manifold)
        {
            manifold->normal = dir.x > b2Scalar(0.0) ? b2Vec2(b2Scalar(1.0), b2Scalar(0.0)) : b2Vec2(b2Scalar(-1.0), b2Scalar(0.0));
            b2Scalar l = b2Sqrt(distSqr);
            if (l > b2_epsilon)
                manifold->normal = dir / l;
            manifold->normal = -manifold->normal;
            manifold->separation = -l - radius;
        }
    }
    else 
    {
        bool collision = distSqr <= radius * radius;
        if (!collision && separationStop)
            return collision;
        if (manifold)
        {
            manifold->normal = dir.x > b2Scalar(0.0) ? b2Vec2(b2Scalar(1.0), b2Scalar(0.0)) : b2Vec2(b2Scalar(-1.0), b2Scalar(0.0));
            b2Scalar l = b2Sqrt(distSqr);
            if (l > b2_epsilon)
                manifold->normal = dir / l;
            manifold->separation = l - radius;
        }
    }

    if (manifold)
    {
        manifold->point = point - rB * manifold->normal;
        manifold->normal = b2Mul(xf1.q, manifold->normal);
        manifold->point = b2Mul(xf1, manifold->point);

        if (flip)
        {
            manifold->normal = -manifold->normal;
            manifold->point = manifold->point + manifold->normal * manifold->separation;
        }
    }

    return collision;
}
