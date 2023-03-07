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

#include <new>

b2Shape* b2PolygonShape::Clone(b2BlockAllocator* allocator) const
{
    void* mem = allocator->Allocate(sizeof(b2PolygonShape));
    b2PolygonShape* clone = new (mem) b2PolygonShape;
    *clone = *this;
    return clone;
}

static b2Vec2 ComputeCentroid(const b2Vec2* vs, int count)
{
    //    b2Assert(count >= 3);

    b2Vec2 c(b2Scalar(0.0), b2Scalar(0.0));
    b2Scalar area = b2Scalar(0.0);

    // Get a reference point for forming triangles.
    // Use the first vertex to reduce round-off errors.
    b2Vec2 s = vs[0];

    const b2Scalar inv3 = b2Scalar(1.0) / b2Scalar(3.0);

    for (int i = 0; i < count; ++i)
    {
        // Triangle vertices.
        b2Vec2 p1 = vs[0] - s;
        b2Vec2 p2 = vs[i] - s;
        b2Vec2 p3 = i + 1 < count ? vs[i+1] - s : vs[0] - s;

        b2Vec2 e1 = p2 - p1;
        b2Vec2 e2 = p3 - p1;

        b2Scalar D = b2Cross(e1, e2);

        b2Scalar triangleArea = b2Scalar(0.5) * D;
        area += triangleArea;

        // Area weighted centroid
        c += triangleArea * inv3 * (p1 + p2 + p3);
    }

    // Centroid
    //    b2Assert(area > b2_epsilon);
    c = (b2Scalar(1.0) / area) * c + s;
    return c;
}

void b2PolygonShape::Set(const b2Vec2* vertices, int count)
{
    //    b2Assert(3 <= count && count <= b2_maxPolygonVertices);
    if (count < 3)
        return;

    m_count = count;

    // Copy vertices.
    for (int i = 0; i < m_count; ++i)
    {
        m_vertices[i] = vertices[i];
    }

    // Compute normals. Ensure the edges have non-zero length.
    for (int i = 0; i < m_count; ++i)
    {
        int i1 = i;
        int i2 = i + 1 < m_count ? i + 1 : 0;
        b2Vec2 edge = m_vertices[i2] - m_vertices[i1];
        //        b2Assert(edge.LengthSquared() > b2_epsilon * b2_epsilon);
        m_normals[i] = b2Cross(edge, b2Scalar(1.0));
        m_normals[i].Normalize();
    }

    // Compute the polygon centroid.
    m_centroid = ComputeCentroid(m_vertices, m_count);
}

bool b2PolygonShape::TestPoint(const b2Transform& xf, const b2Vec2& p) const
{
    b2Vec2 pLocal = b2MulT(xf.q, p - xf.p);
    for (int i = 0; i < m_count; ++i)
    {
        b2Scalar dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
        if (dot > m_radius)
            return false;
    }
    return true;
}

void b2PolygonShape::ComputeAABB(b2AABB* aabb, const b2Transform& xf) const
{
    b2Vec2 lower = b2Mul(xf, m_vertices[0]);
    b2Vec2 upper = lower;
    for (int i = 1; i < m_count; ++i)
    {
        b2Vec2 v = b2Mul(xf, m_vertices[i]);
        lower = b2Min(lower, v);
        upper = b2Max(upper, v);
    }
    b2Vec2 r(m_radius, m_radius);
    aabb->lowerBound = lower - r;
    aabb->upperBound = upper + r;
}

b2Scalar b2PolygonShape::ComputeArea() const
{
    //    b2Assert(m_count >= 3);
    b2Scalar area = b2Scalar(0.0);
    // Get a reference point for forming triangles.
    // Use the first vertex to reduce round-off errors.
    b2Vec2 s = m_vertices[0];
    for (int i = 0; i < m_count; ++i)
    {
        // Triangle vertices.
        b2Vec2 e1 = m_vertices[i] - s;
        b2Vec2 e2 = i + 1 < m_count ? m_vertices[i+1] - s : m_vertices[0] - s;
        b2Scalar D = b2Cross(e1, e2);
        b2Scalar triangleArea = b2Scalar(0.5) * D;
        area += triangleArea;
    }

    return area;
}

bool b2PolygonShape::Validate() const
{
    for (int i = 0; i < m_count; ++i)
    {
        int i1 = i;
        int i2 = i < m_count - 1 ? i1 + 1 : 0;
        b2Vec2 p = m_vertices[i1];
        b2Vec2 e = m_vertices[i2] - p;

        for (int j = 0; j < m_count; ++j)
        {
            if (j == i1 || j == i2)
            {
                continue;
            }

            b2Vec2 v = m_vertices[j] - p;
            b2Scalar c = b2Cross(e, v);
            if (c < b2Scalar(0.0))
            {
                return false;
            }
        }
    }

    return true;
}

bool b2PolygonShape::InscribedSphereAtPoint(const b2Vec2& inp_, const b2Vec2& bdp_, const b2Vec2& normal_, b2Vec2& local_center, b2Scalar &radius) const
{
    b2Scalar eps = b2Scalar(1.e4) * b2_epsilon;
    b2Scalar maxSeparation = b2_maxFloat, tempd;
    b2Vec2 temp; // the first inner point
    int count = 0;
    int bestIndex = 0;
    b2Vec2 inp = inp_;
    b2Vec2 bdp = bdp_;
    b2Vec2 normal = normal_;
    b2Scalar s(0.0), maxS = b2Distance(inp, bdp) - eps;
    while (s < maxS)
    {
        local_center = inp + s * normal;
        if (s > b2Scalar(0.0) && maxSeparation >= -eps && b2Dot(m_normals[bestIndex], local_center - m_vertices[bestIndex]) >= -eps) // the point is still outside the edge normal
        {
            s = (s + maxS) / b2Scalar(2.0);
            continue;
        }
        maxSeparation = -b2_maxFloat; // test the local_center if in the polygon
        for (int i = 0; i < m_count && maxSeparation < -eps; ++i)
        {
            b2Scalar dot = b2Dot(m_normals[i], local_center - m_vertices[i]);
            if (dot > maxSeparation)
            {
                maxSeparation = dot;
                bestIndex = i;
            }
        }
        if (maxSeparation < -eps) // inner point
        {
            if (s == b2Scalar(0.0) && b2Dot(m_normals[bestIndex], normal) >= -eps)
                break;
            count++;
            if (count == 2)
            {
                if (maxSeparation > tempd)
                {
                    maxSeparation = tempd;
                    local_center = temp; 
                }
                break;
            }
            tempd = maxSeparation;
            temp = local_center;
        }
        else if (count ==0)// find the deepest penetration point on the normal m_normals[bestIndex]
        {
            int bi = 0;
            b2Scalar bis = b2_maxFloat;
            int v1 = bestIndex, v2 = v1 + 1;
            if (v2 == m_count)
                v2 = 0;
            for (int i = 0; i < m_count; ++i)
            {
                if (i == v1 || i == v2)
                    continue;
                b2Scalar dot = b2Dot(m_normals[bestIndex], m_vertices[i] - m_vertices[bestIndex]);
                if (dot < bis)
                {
                    bis = dot;
                    bdp = m_vertices[i];
                    bi = 1;
                }
                else if (dot <= bis + eps)
                {
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
            normal.Normalize();
        }
        s = (s + maxS) / b2Scalar(2.0);
    }
    if (maxSeparation < -eps)
    {
        radius = -maxSeparation;
        return true;
    }
    return false;
}

b2Vec2 b2PolygonShape::SupportPoint(const b2Vec2& dir) const
{
    int best = 0;
    b2Scalar res = b2Dot(dir, m_vertices[0]);
    for (int i = 1; i < m_count; ++i)
    {
        b2Scalar d = b2Dot(dir, m_vertices[i]);
        if (d > res)
        {
            best = i;
            res = d;
        }
    }
    return m_vertices[best];
}