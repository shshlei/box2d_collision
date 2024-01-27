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

#include "box2d_collision/b2_shape_polygon.h"

#include "box2d_collision/b2_block_allocator.h"
#include "box2d_collision/b2_math.h"

#include <new>

b2PolygonShape::b2PolygonShape()
{
  m_type = e_polygon;
  m_count = 0;
}

void b2PolygonShape::Set(const b2Vec2 * vertices, int count)
{
  if (count < 3) return;

  m_count = count;

  // Compute normals. Ensure the edges have non-zero length.
  for (int i = 0; i < m_count; ++i) {
    m_vertices[i] = vertices[i];
    int i2 = (i + 1 < m_count ? i + 1 : 0);
    b2Vec2 edge = vertices[i2] - vertices[i];
    m_normals[i] = b2Cross(edge, b2Scalar(1.0));
    m_normals[i].normalize();
  }
}

b2Scalar b2PolygonShape::ComputeArea() const
{
  b2Scalar area = b2Scalar(0.0);
  const b2Vec2 & s = m_vertices[0];
  b2Vec2 e2 = m_vertices[1] - s;
  for (int i = 1; i < m_count - 1; ++i) {
    // Triangle vertices.
    b2Vec2 e1 = e2;
    e2 = m_vertices[i + 1] - s;
    area += b2Cross(e1, e2);
  }
  return b2Scalar(0.5) * area;
}

int b2PolygonShape::GetVerticesCount() const
{
  return m_count;
}

const b2Vec2 * b2PolygonShape::GetVertices() const
{
  return m_vertices;
}

const b2Vec2 * b2PolygonShape::GetNormals() const
{
  return m_normals;
}

const b2Vec2 & b2PolygonShape::GetVertice(int index) const
{
  return m_vertices[index];
}

const b2Vec2 & b2PolygonShape::GetNormal(int index) const
{
  return m_normals[index];
}

b2Shape * b2PolygonShape::Clone(b2BlockAllocator * allocator) const
{
  void * mem = allocator->Allocate(sizeof(b2PolygonShape));
  b2PolygonShape * clone = new (mem) b2PolygonShape;
  *clone = *this;
  return clone;
}

bool b2PolygonShape::TestPoint(const b2Transform & xf, const b2Vec2 & p) const
{
  b2Vec2 pLocal = b2MulT(xf, p);
  for (int i = 0; i < m_count; ++i) {
    b2Scalar dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
    if (dot > b2Scalar(0.0)) return false;
  }
  return true;
}

void b2PolygonShape::ComputeAABB(b2AABB * aabb, const b2Transform & xf) const
{
  b2Vec2 lower = b2Mul(xf, m_vertices[0]);
  b2Vec2 upper = lower;
  for (int i = 1; i < m_count; ++i) {
    b2Vec2 v = b2Mul(xf, m_vertices[i]);
    lower = b2Min(lower, v);
    upper = b2Max(upper, v);
  }
  aabb->min() = lower;
  aabb->max() = upper;
}

bool b2PolygonShape::InscribedSphereAtPoint(const b2Vec2 & inp_, const b2Vec2 & bdp_, const b2Vec2 & normal_, b2Vec2 & local_center, b2Scalar & radius) const
{
  b2Scalar eps = b2Scalar(1.e4) * B2_EPSILON;
  b2Scalar maxSeparation = B2_INFINITY, tempd;
  b2Vec2 temp;  // the first inner point
  int count = 0;
  int bestIndex = 0;
  b2Vec2 inp = inp_;
  b2Vec2 bdp = bdp_;
  b2Vec2 normal = normal_;
  b2Scalar s(0.0), maxS = b2Distance(inp, bdp) - eps;
  while (s < maxS) {
    local_center = inp + s * normal;
    if (s > b2Scalar(0.0) && maxSeparation >= -eps && b2Dot(m_normals[bestIndex], local_center - m_vertices[bestIndex]) >= -eps)  // the point is still outside the edge normal
    {
      s = (s + maxS) / b2Scalar(2.0);
      continue;
    }
    maxSeparation = -B2_INFINITY;  // test the local_center if in the polygon
    for (int i = 0; i < m_count && maxSeparation < -eps; ++i) {
      b2Scalar dot = b2Dot(m_normals[i], local_center - m_vertices[i]);
      if (dot > maxSeparation) {
        maxSeparation = dot;
        bestIndex = i;
      }
    }
    if (maxSeparation < -eps)  // inner point
    {
      if (s == b2Scalar(0.0) && b2Dot(m_normals[bestIndex], normal) >= -eps)
        break;
      count++;
      if (count == 2) {
        if (maxSeparation > tempd) {
          maxSeparation = tempd;
          local_center = temp;
        }
        break;
      }
      tempd = maxSeparation;
      temp = local_center;
    }
    else if (count == 0)  // find the deepest penetration point on the normal m_normals[bestIndex]
    {
      int bi = 0;
      b2Scalar bis = B2_INFINITY;
      int v1 = bestIndex, v2 = v1 + 1;
      if (v2 == m_count)
        v2 = 0;
      for (int i = 0; i < m_count; ++i) {
        if (i == v1 || i == v2)
          continue;
        b2Scalar dot = b2Dot(m_normals[bestIndex], m_vertices[i] - m_vertices[bestIndex]);
        if (dot < bis) {
          bis = dot;
          bdp = m_vertices[i];
          bi = 1;
        }
        else if (dot <= bis + eps) {
          bdp += m_vertices[i];
          bi++;
        }
      }
      if (bis >= -eps)
        break;
      if (bi > 1)
        bdp /= bi;
      inp = local_center;
      s = b2Scalar(0.0);
      maxS = b2Distance(inp, bdp) - eps;
      normal = bdp - inp;
      normal.normalize();
    }
    s = (s + maxS) / b2Scalar(2.0);
  }
  if (maxSeparation < -eps) {
    radius = -maxSeparation;
    return true;
  }
  return false;
}

b2Vec2 b2PolygonShape::SupportPoint(const b2Vec2 & dir) const
{
  int best = 0;
  b2Scalar res = b2Dot(dir, m_vertices[0]);
  for (int i = 1; i < m_count; ++i) {
    b2Scalar d = b2Dot(dir, m_vertices[i]);
    if (d > res) {
      best = i;
      res = d;
    }
  }
  return m_vertices[best];
}
