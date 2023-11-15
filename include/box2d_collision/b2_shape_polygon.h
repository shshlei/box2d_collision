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

#ifndef B2_POLYGON_SHAPE_H
#define B2_POLYGON_SHAPE_H

#include "b2_shape.h"

/// A solid convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to 8.
/// In most cases you should not need many vertices for a convex polygon.
class B2_API b2PolygonShape : public b2Shape
{
public:
  b2PolygonShape();

  /// Create a convex hull from the given array of local points.
  /// The count must be in the range [3, 8].
  /// @warning the points may be re-ordered, even if they form a convex polygon
  /// @warning collinear points are handled but not removed. Collinear points
  /// may lead to poor stacking behavior.
  void Set(const b2Vec2 * points, int count);

  b2Scalar ComputeArea() const;

  int GetVerticesCount() const;

  const b2Vec2 * GetVertices() const;

  const b2Vec2 * GetNormals() const;

  const b2Vec2 & GetVertice(int index) const;

  const b2Vec2 & GetNormal(int index) const;

  /// Implement b2Shape.
  b2Shape * Clone(b2BlockAllocator * allocator) const override;

  /// @see b2Shape::TestPoint
  bool TestPoint(const b2Transform & transform, const b2Vec2 & p) const override;

  /// @see b2Shape::ComputeAABB
  void ComputeAABB(b2AABB * aabb, const b2Transform & transform) const override;

  bool InscribedSphereAtPoint(const b2Vec2 & inp, const b2Vec2 & bdp, const b2Vec2 & normal, b2Vec2 & local_center, b2Scalar & radius) const override;

  b2Vec2 SupportPoint(const b2Vec2 & dir) const override;

private:
  b2Vec2 m_vertices[8];
  b2Vec2 m_normals[8];
  int m_count;
};

#endif
