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

#include "box2d_collision/b2_math.h"
#include "box2d_collision/b2_shape_capsule.h"
#include "box2d_collision/b2_shape_circle.h"
#include "box2d_collision/b2_shape_ellipse.h"
#include "box2d_collision/b2_shape_polygon.h"
#include "box2d_collision/b2_shape_rectangle.h"

#include <string>

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
  unsigned int shape_id[2];
  b2Vec2 normal;
  b2Vec2 points[2];
  b2Vec2 local_points[2];
  b2Transform transforms[2];
  b2Scalar separation;
};

/// This is used to compute the current state of a contact manifold.
struct B2_API b2InscribedSpheres
{
  void Initialize(const b2ContactResult * manifold, const b2Shape * shape1, const b2Shape * shape2);
  void Initialize(const b2Manifold * manifold, const b2Shape * shape1, const b2Transform & xf1, const b2Shape * shape2, const b2Transform & xf2);
  bool has_sphere1, has_sphere2;
  b2Vec2 local_center1, local_center2;
  b2Vec2 center1, center2;
  b2Scalar radius1, radius2;
};

/// Compute the collision manifold between two shapes.
B2_API bool b2CollideShapes(b2Manifold * manifold,
  const b2Shape * shapeA, const b2Transform & xfA,
  const b2Shape * shapeB, const b2Transform & xfB, bool separationStop = true);

/// Determine if two generic shapes overlap.
B2_API bool b2CollideShapes(const b2Shape * shapeA, const b2Transform & xfA, const b2Shape * shapeB, const b2Transform & xfB);

/// Compute the collision manifold between two circles.
B2_API bool b2CollideCircles(b2Manifold * manifold,
  const b2CircleShape * circleA, const b2Transform & xfA,
  const b2CircleShape * circleB, const b2Transform & xfB, bool separationStop = true);

B2_API bool b2CollideCapsuleCircle(b2Manifold * manifold,
  const b2CapsuleShape * capsuleA, const b2Transform & xfA,
  const b2CircleShape * circleB, const b2Transform & xfB, bool separationStop);

/// Compute the collision manifold between a rectangle and a circle.
B2_API bool b2CollideRectangleCircle(b2Manifold * manifold,
  const b2RectangleShape * rectA, const b2Transform & xfA,
  const b2CircleShape * circleB, const b2Transform & xfB, bool separationStop);

B2_API bool b2CollideCapsules(b2Manifold * manifold,
  const b2CapsuleShape * capsuleA, const b2Transform & xfA,
  const b2CapsuleShape * capsuleB, const b2Transform & xfB, bool separationStop);

/// Compute the collision manifold between a polygon and a circle.
B2_API bool b2CollidePolygonCircle(b2Manifold * manifold,
  const b2PolygonShape * polygonA, const b2Transform & xfA,
  const b2CircleShape * circleB, const b2Transform & xfB, bool separationStop = true);

/// Compute the collision manifold between two polygons.
B2_API bool b2CollidePolygons(b2Manifold * manifold,
  const b2PolygonShape * polygonA, const b2Transform & xfA,
  const b2PolygonShape * polygonB, const b2Transform & xfB, bool separationStop = true);

#endif
