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

#ifndef B2_CALLBACKS_H
#define B2_CALLBACKS_H

#include "b2_api.h"
#include "b2_settings.h"

struct b2Vec2;
struct b2Transform;
class b2Fixture;
class b2Body;
class b2Contact;
struct b2Manifold;

/// Implement this class to provide collision filtering. In other words, you can implement
/// this class if you want finer control over contact creation.
class B2_API b2ContactFilter
{
public:
    virtual ~b2ContactFilter() {}

    /// Return true if contact calculations should be performed between these two shapes.
    /// @warning for performance reasons this is only called when the AABBs begin to overlap.
    virtual bool ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB);
};

/// Callback class for AABB queries.
/// See b2BVHManager::Query
class B2_API b2QueryCallback
{
public:
    virtual ~b2QueryCallback() {}

    /// Called for each fixture found in the query AABB.
    /// @return false to terminate the query.
    virtual bool ReportFixture(b2Fixture* fixture) = 0;
};

class B2_API b2QueryCallback2
{
public:
    virtual ~b2QueryCallback2() {}

    /// Called for each fixture found in the query AABB.
    /// @return false to terminate the query.
    virtual bool ReportFixture(b2Fixture* fixtureA, b2Fixture* fixtureB) = 0;
};

/// Callback class for ray casts.
/// See b2BVHManager::RayCast
class B2_API b2RayCastCallback
{
public:
    virtual ~b2RayCastCallback() {}

    /// Called for each fixture found in the query. You control how the ray cast
    /// proceeds by returning a float:
    /// return -1: ignore this fixture and continue
    /// return 0: terminate the ray cast
    /// return fraction: clip the ray to this point
    /// return 1: don't clip the ray and continue
    /// @param fixture the fixture hit by the ray
    /// @param point the point of initial intersection
    /// @param normal the normal vector at the point of intersection
    /// @param fraction the fraction along the ray at the point of intersection
    /// @return -1 to filter, 0 to terminate, fraction to clip the ray for
    /// closest hit, 1 to continue
    virtual float ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float fraction) = 0;
};

#endif
