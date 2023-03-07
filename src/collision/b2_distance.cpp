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

#include "box2d_collision/b2_distance.h"

namespace
{
bool doSimplex2(const b2Vec2 &a, const b2Vec2 &b, b2Vec2 &dir)
{
    b2Vec2 ab = b - a;
    b2Scalar sgn = b2Cross(b, ab);
    if (sgn * sgn < ab.LengthSquared() * b.LengthSquared() * b2_epsilon2)
        return true;
    if (sgn > b2Scalar(0.0))
        // Origin is left of ab.
        dir = b2Cross(b2Scalar(1.0), ab).Normalized();
    else
        // Origin is right of ab.
        dir = b2Cross(ab, b2Scalar(1.0)).Normalized();
    return false;
}

bool doSimplex2(b2Simplex &simplex, b2Vec2 &dir)
{
    return doSimplex2(simplex.At(1).v, simplex.At(0).v, dir);
}

bool doSimplex3(b2Simplex &simplex, b2Vec2 &dir)
{
    const b2Support &a = simplex.At(2), &b = simplex.At(1), &c = simplex.At(0);

    b2Vec2 ab = b.v - a.v;
    b2Scalar sgn1 = b2Cross(b.v, ab);
    if (sgn1 * sgn1 < ab.LengthSquared() * b.v.LengthSquared() * b2_epsilon2)
    {
        simplex.Set(0, b);
        simplex.Set(1, a);
        simplex.SetSize(2);
        return true;
    }

    b2Vec2 ac = c.v - a.v;
    b2Scalar sgn2 = b2Cross(c.v, ac);
    if (sgn2 * sgn2 < ac.LengthSquared() * c.v.LengthSquared() * b2_epsilon2)
    {
        simplex.Set(0, c);
        simplex.Set(1, a);
        simplex.SetSize(2);
        return true;
    }

    b2Vec2 dir1, dir2;
    if (sgn1 > b2Scalar(0.0))
        dir1 = b2Cross(b2Scalar(1.0), ab).Normalized();
    else
        dir1 = b2Cross(ab, b2Scalar(1.0)).Normalized();
    if (sgn2 > b2Scalar(0.0))
        dir2 = b2Cross(b2Scalar(1.0), ac).Normalized();
    else
        dir2 = b2Cross(ac, b2Scalar(1.0)).Normalized();

    b2Scalar dot1 = b2Dot(dir1, a.v); 
    b2Scalar dot2 = b2Dot(dir2, a.v);
    if (dot1 < b2Scalar(0.0) && dot2 < b2Scalar(0.0))
        return true;
    else if (dot1 > b2Scalar(0.0)) 
    {
        simplex.Set(0, b);
        simplex.Set(1, a);
        simplex.SetSize(2);
        dir = dir1;
    }
    else 
    {
        simplex.Set(0, c);
        simplex.Set(1, a);
        simplex.SetSize(2);
        dir = dir2;
    }
    return false;
}

bool doSimplex(b2Simplex &simplex, b2Vec2 &dir)
{
    if (simplex.Size() == 2)
        return doSimplex2(simplex, dir);
    else
        return doSimplex3(simplex, dir);
}

b2Scalar originSegmentClosestPoint(const b2Vec2 &w1, const b2Vec2 &w2, b2Vec2 &closest_p)
{
    b2Vec2 e12 = w2 - w1;

    // w1 region
    b2Scalar d12_2 = -b2Dot(w1, e12);
    if (d12_2 <= b2Scalar(0.0))
    {
        closest_p = w1;
        return closest_p.Length();
    }

    // w2 region
    b2Scalar d12_1 = b2Dot(w2, e12);
    if (d12_1 <= b2Scalar(0.0))
    {
        closest_p = w2;
        return closest_p.Length();
    }

    // Must be in e12 region.
    b2Scalar inv_d12 = b2Scalar(1.0) / (d12_1 + d12_2);
    b2Scalar alpha = d12_1 * inv_d12;
    closest_p = alpha * w1 + (b2Scalar(1.0) - alpha) * w2;
    return closest_p.Length();
}

b2Scalar originSegmentClosestPoint(const b2Vec2 &w1, const b2Vec2 &w2, b2Scalar &alpha)
{
    b2Vec2 e12 = w2 - w1;

    // w1 region
    b2Scalar d12_2 = -b2Dot(w1, e12);
    if (d12_2 <= b2Scalar(0.0))
    {
        alpha = b2Scalar(0.0);
        return w1.Length();
    }

    // w2 region
    b2Scalar d12_1 = b2Dot(w2, e12);
    if (d12_1 <= b2Scalar(0.0))
    {
        alpha = b2Scalar(1.0);
        return w2.Length();
    }

    // Must be in e12 region.
    b2Scalar inv_d12 = b2Scalar(1.0) / (d12_1 + d12_2);
    alpha = d12_1 * inv_d12;
    b2Vec2 closest_p = alpha * w1 + (b2Scalar(1.0) - alpha) * w2;
    return closest_p.Length();
}

b2Scalar simplexReduceToSegment(b2Simplex &simplex, b2Scalar &dist, b2Vec2 &best_witness)
{
    b2Scalar newdist;
    b2Vec2 witness;
    int best = -1;

    // try the third point in all two positions
    for (int i = 0; i < 2; i++)
    {
        newdist = originSegmentClosestPoint(simplex.At(i == 0 ? 2 : 0).v, simplex.At(i == 1 ? 2 : 1).v, witness);
        // record the best triangle
        if (newdist < dist)
        {
            best = i;
            dist = newdist;
            best_witness = witness;
        }
    }
    if (best >= 0)
        simplex.Set(best, simplex.At(2));
    simplex.SetSize(2);
    return dist;
}

void simplexClosestP(b2Simplex &simplex, b2Vec2 &closest_p, b2Scalar &distp)
{
    int sz = simplex.Size();
    if (sz == 1)
    {
        closest_p = simplex.At(0).v;
        distp = closest_p.Length();
    }
    else if (sz == 2)
        distp = originSegmentClosestPoint(simplex.At(0).v, simplex.At(1).v, closest_p);
    else
        distp = simplexReduceToSegment(simplex, distp, closest_p);
}

void support(const b2MinkowskiDiff& shape, const b2Vec2 &closest_p, b2Vec2 &dir, b2Support &last)
{
    // point direction towards the origin
    dir = (-closest_p).Normalized();
    last = shape.SupportS(dir);
}

bool ccdOptimal(const b2Vec2 &closest_p, const b2Vec2 &dir, const b2Vec2 &last, b2Scalar tol)
{
    b2Vec2 temp = last - closest_p;
    if (b2Dot(dir, temp) < tol)
        return true;
    return false;
}

void extractObjectPointsFromPoint(const b2Support &q, b2Vec2 &p1, b2Vec2 &p2)
{
    p1 = q.v1;
    p2 = q.v2;
}

b2Scalar alphaRatio(const b2Support &a, const b2Support &b, const b2Vec2 &p)
{
    b2Vec2 AB = b.v - a.v;

    b2Scalar abs_AB_x{std::abs(AB(0))};
    b2Scalar abs_AB_y{std::abs(AB(1))};

    b2Scalar A_i, AB_i, p_i;
    if (abs_AB_x >= abs_AB_y)
    {
        A_i = a.v(0);
        AB_i = AB(0);
        p_i = p(0);
    }
    else
    {
        A_i = a.v(1);
        AB_i = AB(1);
        p_i = p(1);
    }
    if (std::abs(AB_i) < b2_epsilon)
        return b2Scalar(-1.0);
    return (p_i - A_i) / AB_i;
}

void extractObjectPointsFromSegment(const b2Support &a, const b2Support &b, b2Vec2 &p1, b2Vec2 &p2, const b2Vec2 &p)
{
    b2Scalar s = alphaRatio(a, b, p);
    if (s < b2Scalar(0.0)) 
    {
        // Points are coincident; treat as a single point.
        extractObjectPointsFromPoint(a, p1, p2);
        return;
    }

    auto calc_p = [](const b2Vec2 &p_a, const b2Vec2 &p_b, b2Vec2 &p, b2Scalar s) {
        p = p_a + s * (p_b - p_a);
    };

    calc_p(a.v1, b.v1, p1, s);
    calc_p(a.v2, b.v2, p2, s);
}

void extractClosestPoints(const b2Simplex &simplex, b2Vec2 &p1, b2Vec2 &p2, const b2Vec2 &p)
{
    const int simplex_size = simplex.Size();
    if (simplex_size == 1)
        extractObjectPointsFromPoint(simplex.At(0), p1, p2);
    else
        extractObjectPointsFromSegment(simplex.At(0), simplex.At(1), p1, p2, p);
}

bool ccdSeparation(const b2MinkowskiDiff &shape, b2Simplex &simplex, int max_iterations, b2Scalar tol)
{
    b2Vec2 dir(1.0, 0.0); // direction vector
    b2Support last = shape.SupportS(dir); // last support point
    simplex.Add(last);
    if (b2Dot(last.v, dir) < b2Scalar(0.0))
        return true;
    dir = (-last.v).Normalized();
    for (int iterations = 0; iterations < max_iterations; ++iterations)
    {
        // obtain support point
        last = shape.SupportS(dir);
        simplex.Add(last);

        // check if farthest point in Minkowski difference in direction dir
        // isn't somewhere before origin (the test on negative dot product)
        // - because if it is, objects are not intersecting at all.
        if (b2Dot(last.v, dir) < b2Scalar(0.0))
            return true; // intersection not found
        if (doSimplex(simplex, dir))
            return false; // intersection found
    }
    return true;
}

b2Scalar ccdDistance(const b2MinkowskiDiff &shape, b2Simplex &simplex, int max_iterations, b2Scalar tol, b2Vec2 &p1, b2Vec2 &p2)
{
    b2Scalar last_dist = b2_maxFloat;
    b2Vec2 closest_p; // The point on the simplex that is closest to the
    for (int iterations = 0; iterations < max_iterations; ++iterations)
    {
        // origin.
        // get a next direction vector
        // we are trying to find out a point on the minkowski difference
        // that is nearest to the origin, so we obtain a point on the
        // simplex that is nearest and try to exapand the simplex towards
        // the origin
        simplexClosestP(simplex, closest_p, last_dist);

        b2Vec2 dir; // direction vector
        b2Support last; // last support point
        support(shape, closest_p, dir, last);

        if (ccdOptimal(closest_p, dir, last.v, tol))
        {
            extractClosestPoints(simplex, p1, p2, closest_p);
            return last_dist;
        }
        simplex.Add(last);
    }
    simplexClosestP(simplex, closest_p, last_dist);
    extractClosestPoints(simplex, p1, p2, closest_p);
    return last_dist;
}

bool simplexToPolytope2(const b2MinkowskiDiff &shape, b2Simplex &simplex, b2Polytope &polytope)
{
    b2Support a = simplex.At(1);
    if (a.v.LengthSquared() < b2_epsilon2)
    {
        simplex.Set(0, a);
        simplex.SetSize(1);
        return false;
    }
    b2Support b = simplex.At(0);
    b2Vec2 ab = b.v - a.v;

    b2Vec2 dir1 = b2Cross(b2Scalar(1.0), ab).Normalized();
    b2Support last1 = shape.SupportS(dir1);
    if (b2Dot(dir1, last1.v) < b2_epsilon)
        return false;

    b2Vec2 dir2 = b2Cross(ab, b2Scalar(1.0)).Normalized();
    b2Support last2 = shape.SupportS(dir2);
    if (b2Dot(dir2, last2.v) < b2_epsilon)
        return false;

    if (b2Dot(ab, last1.v - a.v) > b2Scalar(0.0))
    {
        polytope.ps.push_back(b);
        polytope.ps.push_back(last1);
        polytope.ps.push_back(a);
        polytope.ps.push_back(last2);
    }
    else 
    {
        polytope.ps.push_back(b);
        polytope.ps.push_back(last2);
        polytope.ps.push_back(a);
        polytope.ps.push_back(last1);
    }
    return true;
}

bool simplexToPolytope3(b2Simplex &simplex, b2Polytope &polytope)
{
    b2Support a = simplex.At(2);
    if (a.v.LengthSquared() < b2_epsilon2)
    {
        simplex.Set(0, a);
        simplex.SetSize(1);
        return false;
    }
    b2Support b = simplex.At(1), c = simplex.At(0);
    if (b2Cross(b.v, a.v) > b2Scalar(0.0))
    {
        polytope.ps.push_back(c);
        polytope.ps.push_back(b);
        polytope.ps.push_back(a);
    }
    else
    {
        polytope.ps.push_back(c);
        polytope.ps.push_back(a);
        polytope.ps.push_back(b);
    }
    return true;
}

bool simplexToPolytope(const b2MinkowskiDiff &shape, b2Simplex &simplex, b2Polytope &polytope)
{
    int sz = simplex.Size();
    if (sz == 2)
        return simplexToPolytope2(shape, simplex, polytope);
    else if (sz == 3)
        return simplexToPolytope3(simplex, polytope);
    else 
        b2Error("The simplex to polytope function has a wrong number of point size!");
    return true;
}

b2Scalar ccdPenetration(const b2MinkowskiDiff &shape, b2Simplex &simplex, int max_iterations, b2Scalar tol, b2Vec2 &p1, b2Vec2 &p2)
{
    b2Polytope polytope;
    if (!simplexToPolytope(shape, simplex, polytope))
    {
        extractClosestPoints(simplex, p1, p2, b2Vec2_zero);
        return b2Scalar(0.0);
    }

    for (std::size_t i = 0, sz = polytope.ps.size(); i < sz; i++)
    {
        std::size_t j = (i + 1) % sz;
        b2Polytope::Edge edge(i, j);
        edge.dist = originSegmentClosestPoint(polytope.ps[i].v, polytope.ps[j].v, edge.s);
        polytope.edgeQueue.push(edge);
    }

    auto calc_p = [](const b2Vec2 &p_a, const b2Vec2 &p_b, b2Vec2 &p, b2Scalar s) {
        p = p_a + s * (p_b - p_a);
    };

    b2Polytope::Edge edge = polytope.edgeQueue.top();
    for (int iterations = 0; iterations < max_iterations; ++iterations)
    {
        edge = polytope.edgeQueue.top();
        polytope.edgeQueue.pop();
        std::size_t i = edge.index1, j = edge.index2;
        const b2Vec2& vi = polytope.ps[i].v;
        const b2Vec2& vj = polytope.ps[j].v;
        if (vi.IsApprox(vj, tol))
        {
            calc_p(polytope.ps[i].v1, polytope.ps[j].v1, p1, edge.s);
            calc_p(polytope.ps[i].v2, polytope.ps[j].v2, p2, edge.s);
            return edge.dist;
        }

        b2Vec2 dir = b2Cross(vj - vi, b2Scalar(1.0)).Normalized();
        b2Support last = shape.SupportS(dir);
        if (b2Dot(dir, last.v - vi) < tol)
        {
            calc_p(polytope.ps[i].v1, polytope.ps[j].v1, p1, edge.s);
            calc_p(polytope.ps[i].v2, polytope.ps[j].v2, p2, edge.s);
            return edge.dist;
        }
        polytope.ps.push_back(last);
        std::size_t index = polytope.ps.size() - 1;
        edge.Set(i, index);
        edge.dist = originSegmentClosestPoint(vi, last.v, edge.s);
        polytope.edgeQueue.push(edge);
        edge.Set(index, j);
        edge.dist = originSegmentClosestPoint(last.v, vj, edge.s);
        polytope.edgeQueue.push(edge);
    }
    edge = polytope.edgeQueue.top();
    std::size_t i = edge.index1, j = edge.index2;
    calc_p(polytope.ps[i].v1, polytope.ps[j].v1, p1, edge.s);
    calc_p(polytope.ps[i].v2, polytope.ps[j].v2, p2, edge.s);
    return edge.dist;
}
}

bool b2ShapeDistance::Separation(const b2Shape* shape1, const b2Transform &xf1,
        const b2Shape* shape2, const b2Transform &xf2) const
{
    b2MinkowskiDiff shape;
    shape.shapes[0] = shape1;
    shape.shapes[1] = shape2;
    shape.toshape1 = b2MulT(xf2.q, xf1.q);
    shape.toshape0 = b2MulT(xf1, xf2);
    b2Simplex simplex;
    return ccdSeparation(shape, simplex, max_distance_iterations, distance_tolerance);
}

bool b2ShapeDistance::Distance(const b2Shape* shape1, const b2Transform &xf1,
        const b2Shape* shape2, const b2Transform &xf2,
        b2Scalar *dist, b2Vec2 *p1, b2Vec2 *p2) const
{
    b2MinkowskiDiff shape;
    shape.shapes[0] = shape1;
    shape.shapes[1] = shape2;
    shape.toshape1 = b2MulT(xf2.q, xf1.q);
    shape.toshape0 = b2MulT(xf1, xf2);

    b2Simplex simplex;
    if (!ccdSeparation(shape, simplex, max_distance_iterations, distance_tolerance))
    {
        if (dist) *dist = b2Scalar(-1.0);
        return false;
    }
    
    b2Vec2 p1_, p2_;
    b2Scalar d = ccdDistance(shape, simplex, max_distance_iterations, distance_tolerance, p1_, p2_);
    if (dist) *dist = d;
    if (d > b2Scalar(0.0))
    {
        if (p1) *p1 = b2Mul(xf1, p1_);
        if (p2) *p2 = b2Mul(xf1, p2_);
        return true;
    }
    else 
        return false;
}

bool b2ShapeDistance::SignedDistance(const b2Shape* shape1, const b2Transform &xf1,
        const b2Shape* shape2, const b2Transform &xf2,
        b2Scalar *dist, b2Vec2 *p1, b2Vec2 *p2) const
{
    b2MinkowskiDiff shape;
    shape.shapes[0] = shape1;
    shape.shapes[1] = shape2;
    shape.toshape1 = b2MulT(xf2.q, xf1.q);
    shape.toshape0 = b2MulT(xf1, xf2);

    b2Scalar d;
    b2Vec2 p1_, p2_;
    b2Simplex simplex;
    if (ccdSeparation(shape, simplex, max_distance_iterations, distance_tolerance))
        d = ccdDistance(shape, simplex, max_distance_iterations, distance_tolerance, p1_, p2_);
    else 
        d = -ccdPenetration(shape, simplex, max_distance_iterations, distance_tolerance, p1_, p2_);
    if (dist) *dist = d;
    if (p1) *p1 = b2Mul(xf1, p1_);
    if (p2) *p2 = b2Mul(xf1, p2_);
    if (d > b2Scalar(0.0))
        return true;
    else 
        return false;
}
