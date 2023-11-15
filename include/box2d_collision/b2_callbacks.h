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
#include "b2_math.h"

class b2Fixture;
class b2Body;
struct b2Manifold;

/// Implement this class to provide collision filtering. In other words, you can implement
/// this class if you want finer control over contact creation.
class B2_API b2ContactFilter
{
public:
  virtual ~b2ContactFilter() {}

  /// Return true if contact calculations should be performed between these two shapes.
  /// @warning for performance reasons this is only called when the AABBs begin to overlap.
  virtual bool ShouldCollide(b2Fixture * fixtureA, b2Fixture * fixtureB);
};

/// Callback class for AABB queries.
/// See b2BVHManager::Collide
class B2_API b2NaiveCallback
{
public:
  virtual ~b2NaiveCallback() {}

  /// Called for each fixture found in the query AABB.
  /// @return collision.
  virtual bool ReportCollision(b2Fixture * fixture)
  {
    return true;
  }

  /// Collision between two fixtures
  virtual bool ReportCollision(b2Fixture * fixtureA, b2Fixture * fixtureB)
  {
    return true;
  }

  /// @return separation.
  virtual bool ReportDistance(b2Fixture * fixture, b2Scalar & dist)
  {
    return true;
  }

  /// Distance between two fixtures
  virtual bool ReportDistance(b2Fixture * fixtureA, b2Fixture * fixtureB, b2Scalar & dist)
  {
    return true;
  }
};

#endif
