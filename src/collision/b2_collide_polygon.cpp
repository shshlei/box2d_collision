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

// Find the max separation between poly1 and poly2 using edge normals from poly1.
static b2Scalar b2FindMaxSeparation(int * edgeIndex, b2Vec2 * point,
  const b2PolygonShape * poly1, const b2Transform & xf1,
  const b2PolygonShape * poly2, const b2Transform & xf2,
  bool separationStop, bool & collision)
{
  collision = true;
  int count1 = poly1->GetVerticesCount();
  int count2 = poly2->GetVerticesCount();
  const b2Vec2 * n1s = poly1->GetNormals();
  const b2Vec2 * v1s = poly1->GetVertices();
  const b2Vec2 * v2s = poly2->GetVertices();
  b2Transform xf = b2MulT(xf2, xf1);

  int bestIndex = 0;
  b2Scalar maxSeparation = -B2_INFINITY;
  for (int i = 0; i < count1; ++i) {
    // Get poly1 normal in frame2.
    b2Vec2 n = b2Mul(xf.linear(), n1s[i]);
    b2Vec2 v1 = b2Mul(xf, v1s[i]);

    // Find deepest point for normal i.
    b2Scalar si = B2_INFINITY;
    b2Vec2 temp;
    for (int j = 0; j < count2; ++j) {
      b2Scalar sij = n.dot(v2s[j] - v1);
      if (sij < si) {
        si = sij;
        temp = v2s[j];
      }
    }

    if ((maxSeparation <= b2Scalar(0.0) ? si > maxSeparation : si > b2Scalar(0.0)) && si < maxSeparation) {
      maxSeparation = si;
      bestIndex = i;
      *point = temp;
      if (si > b2Scalar(0.0)) {
        collision = false;
        if (separationStop)
          break;
      }
    }
  }

  *edgeIndex = bestIndex;
  return maxSeparation;
}

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on B - return if separation axis is found
// Choose reference edge as min(minA, minB)

// The normal points from 1 to 2
bool b2CollidePolygons(b2Manifold * manifold,
  const b2PolygonShape * polyA, const b2Transform & xfA,
  const b2PolygonShape * polyB, const b2Transform & xfB, bool separationStop)
{
  bool collision = false;
  int edgeA = 0;
  b2Vec2 pointB;
  b2Scalar separationA = b2FindMaxSeparation(&edgeA, &pointB, polyA, xfA, polyB, xfB, (!manifold || separationStop), collision);
  if (!collision && (!manifold || separationStop))
    return false;

  int edgeB = 0;
  b2Vec2 pointA;
  b2Scalar separationB = b2FindMaxSeparation(&edgeB, &pointA, polyB, xfB, polyA, xfA, (!manifold || separationStop), collision);
  if (!collision && (!manifold || separationStop))
    return false;

  const b2PolygonShape * poly1 = polyA;
  b2Transform xf1 = xfA;
  int edge1 = edgeA;
  b2Scalar separation = separationA;
  b2Vec2 c = b2Mul(xfB, pointB);
  bool flip = false;
  if ((separationB > b2Scalar(0.0) ? (separationA < b2Scalar(0.0) || separationB < separationA) : separationB > separationA)) {
    poly1 = polyB;
    xf1 = xfB;
    edge1 = edgeB;
    separation = separationB;
    c = b2Mul(xfA, pointA);
    flip = true;
  }

  collision = false;

  // If the point is inside the polygon ...
  if (separation <= B2_EPSILON) {
    collision = true;
    if (manifold) {
      manifold->separation = separation;
      manifold->normal = b2Mul(xf1.linear(), poly1->GetNormal(edge1));
    }
  }
  else {
    b2Vec2 cLocal = b2MulT(xf1, c);
    // Vertices that subtend the incident face.
    int vertIndex1 = edge1;
    int vertIndex2 = (vertIndex1 + 1 < poly1->GetVerticesCount() ? vertIndex1 + 1 : 0);
    const b2Vec2 & v1 = poly1->GetVertice(vertIndex1);
    const b2Vec2 & v2 = poly1->GetVertice(vertIndex2);
    // Compute barycentric coordinates
    b2Scalar u1 = (cLocal - v1).dot(v2 - v1);
    b2Scalar u2 = (cLocal - v2).dot(v1 - v2);
    if (u1 <= b2Scalar(0.0)) {
      double d = b2DistanceSquared(cLocal, v1);
      if (d <= b2Scalar(0.0))
        collision = true;
      else if (separationStop)
        return collision;
      if (manifold) {
        manifold->separation = sqrt(d);
        manifold->normal = xf1.linear() * (cLocal - v1);
        manifold->normal.normalize();
      }
    }
    else if (u2 <= b2Scalar(0.0)) {
      double d = b2DistanceSquared(cLocal, v2);
      if (d <= b2Scalar(0.0))
        collision = true;
      else if (separationStop)
        return collision;
      if (manifold) {
        manifold->separation = sqrt(d);
        manifold->normal = xf1.linear() * (cLocal - v2);
        manifold->normal.normalize();
      }
    }
    else {
      if (separation <= b2Scalar(0.0))
        collision = true;
      else if (separationStop)
        return collision;
      if (manifold) {
        manifold->separation = separation;
        manifold->normal = xf1.linear() * poly1->GetNormal(edge1);
      }
    }
  }

  if (manifold) {
    manifold->point = c;
    if (flip) {
      manifold->normal = -manifold->normal;
      manifold->point = manifold->point + manifold->normal * manifold->separation;
    }
  }
  return collision;
}
