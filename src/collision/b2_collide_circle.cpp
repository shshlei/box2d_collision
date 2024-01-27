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

bool b2CollideCircles(b2Manifold * manifold,
  const b2CircleShape * circleA, const b2Transform & xfA,
  const b2CircleShape * circleB, const b2Transform & xfB, bool separationStop)
{
  const b2Vec2 & pA = xfA.translation();
  const b2Vec2 & pB = xfB.translation();

  b2Vec2 d = pB - pA;
  b2Scalar distSqr = b2Dot(d, d);
  b2Scalar rA = circleA->GetRadius(), rB = circleB->GetRadius();
  b2Scalar radius = rA + rB;
  bool collision = distSqr <= radius * radius;
  if (!collision && separationStop)
    return collision;
  if (manifold) {
    manifold->normal = b2Vec2(b2Scalar(1.0), b2Scalar(0.0));
    b2Scalar l = sqrt(distSqr);
    if (l > B2_EPSILON)
      manifold->normal = d / l;
    manifold->separation = l - radius;
    manifold->point = pB - rB * manifold->normal;
  }
  return collision;
}

bool b2CollidePolygonCircle(b2Manifold * manifold,
  const b2PolygonShape * polygonA, const b2Transform & xfA,
  const b2CircleShape * circleB, const b2Transform & xfB, bool separationStop)
{
  bool collision = false;
  // Compute circle position in the frame of the polygon.
  const b2Vec2 & c = xfB.translation();
  b2Vec2 cLocal = b2MulT(xfA, c);

  // Find the max separating edge.
  int normalIndex = 0;
  b2Scalar separation = -B2_INFINITY;
  b2Scalar radius = circleB->GetRadius();
  int vertexCount = polygonA->GetVerticesCount();
  const b2Vec2 * vertices = polygonA->GetVertices();
  const b2Vec2 * normals = polygonA->GetNormals();
  for (int i = 0; i < vertexCount; ++i) {
    b2Scalar s = b2Dot(normals[i], cLocal - vertices[i]);
    if ((!manifold || separationStop) && s > radius)  // Early out.
      return false;
    if (s > separation) {
      separation = s;
      normalIndex = i;
    }
  }

  // If the center is inside the polygon ...
  if (separation < B2_EPSILON) {
    if (manifold) {
      manifold->separation = separation - radius;
      manifold->normal = b2Mul(xfA.linear(), normals[normalIndex]);
      manifold->point = c - manifold->normal * circleB->GetRadius();
    }
    return true;
  }

  // Vertices that subtend the incident face.
  int vertIndex1 = normalIndex;
  int vertIndex2 = (vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0);
  const b2Vec2 & v1 = vertices[vertIndex1];
  const b2Vec2 & v2 = vertices[vertIndex2];
  // Compute barycentric coordinates
  b2Vec2 d1 = cLocal - v1, d2 = cLocal - v2;
  b2Scalar u1 = b2Dot(d1, v2 - v1);
  b2Scalar u2 = b2Dot(d2, v1 - v2);
  if (u1 <= b2Scalar(0.0)) {
    b2Scalar d = d1.squaredNorm();
    if (d <= radius * radius)
      collision = true;
    else if (separationStop)
      return false;
    if (manifold) {
      manifold->separation = sqrt(d) - radius;
      manifold->normal = b2Mul(xfA.linear(), d1);
      manifold->normal.normalize();
      manifold->point = c - manifold->normal * circleB->GetRadius();
    }
  }
  else if (u2 <= b2Scalar(0.0)) {
    b2Scalar d = d2.squaredNorm();
    if (d <= radius * radius)
      collision = true;
    else if (separationStop)
      return false;
    if (manifold) {
      manifold->separation = sqrt(d) - radius;
      manifold->normal = b2Mul(xfA.linear(), d2);
      manifold->normal.normalize();
      manifold->point = c - manifold->normal * circleB->GetRadius();
    }
  }
  else {
    if (separation <= radius)
      collision = true;
    else if (separationStop)
      return false;
    if (manifold) {
      manifold->separation = separation - radius;
      manifold->normal = b2Mul(xfA.linear(), normals[normalIndex]);
      manifold->point = c - manifold->normal * circleB->GetRadius();
    }
  }
  return collision;
}

bool b2NearestPointInBox(const b2Vec2 & hsize, const b2Vec2 & q, b2Vec2 & b)
{
  // Clamp the point to the box. If we do *any* clamping we know the center was
  // outside. If we did *no* clamping, the point is inside the box.
  bool clamped = false;
  for (int i = 0; i < 2; ++i) {
    b(i) = q(i);
    if (q(i) < -hsize(i)) {
      clamped = true;
      b(i) = -hsize(i);
    }
    else if (q(i) > hsize(i)) {
      clamped = true;
      b(i) = hsize(i);
    }
  }
  return !clamped;
}

bool b2CollideRectangleCircle(b2Manifold * manifold,
  const b2RectangleShape * rectA, const b2Transform & xfA,
  const b2CircleShape * circleB, const b2Transform & xfB, bool separationStop)
{
  // Find the circle center c in the box's frame.
  const b2Vec2 & c = xfB.translation();
  b2Vec2 cLocal = b2MulT(xfA, c);

  const b2Scalar radius = circleB->GetRadius();

  // Find b, the nearest point *inside* the box to the circle center c
  b2Vec2 b;
  bool collision = b2NearestPointInBox(rectA->GetHalfSides(), cLocal, b);
  if (!collision) {
    b2Vec2 bc = cLocal - b;
    b2Scalar d = bc.squaredNorm();
    if (d <= radius * radius)
      collision = true;
    else if (separationStop)
      return false;
    if (manifold) {
      manifold->separation = sqrt(d) - radius;
      manifold->normal = b2Mul(xfA.linear(), bc);
      manifold->normal.normalize();
      manifold->point = c - manifold->normal * circleB->GetRadius();
    }
  }
  else if (manifold) {
    b2Scalar eps = b2Scalar(16.0) * B2_EPSILON;
    b2Scalar min_distance = B2_INFINITY;
    int min_axis = -1;
    const b2Vec2 & hsize = rectA->GetHalfSides();
    for (int i = 0; i < 2; ++i) {
      b2Scalar dist = (b(i) >= 0.0 ? hsize(i) - b(i) : b(i) + hsize(i));
      // To be closer, the face has to be more than epsilon closer.
      if (dist + eps < min_distance) {
        min_distance = dist;
        min_axis = i;
      }
    }
    manifold->normal.setZero();
    manifold->normal(min_axis) = (b(min_axis) >= 0.0 ? 1.0 : -1.0);
    manifold->normal = b2Mul(xfA.linear(), manifold->normal);
    manifold->separation = -(min_distance + radius);
    manifold->point = c - manifold->normal * circleB->GetRadius();
  }
  // We didn't *prove* separation, so we must be in penetration.
  return collision;
}
