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

#ifndef B2_DISTANCE_H
#define B2_DISTANCE_H

#include "b2_math.h"
#include "b2_shape.h"
#include <vector>
#include <queue>

struct b2Support 
{
    b2Vec2 v, v1, v2;

    b2Support() = default;

    b2Support(const b2Support& s)
    {
        v = s.v; v1 = s.v1; v2 = s.v2; 
    }

    B2_FORCE_INLINE b2Support& operator=(const b2Support& s)
    {
        v = s.v; v1 = s.v1; v2 = s.v2; 
        return *this;
    }
};

struct b2Simplex 
{
    b2Support ps[3];
    int last; //!< index of last added point
    b2Simplex() : last(-1){}

    B2_FORCE_INLINE int Size() const
    {
        return last + 1;
    }
    B2_FORCE_INLINE const b2Support& LastSupport() const
    {
        return ps[last];
    }
    B2_FORCE_INLINE const b2Support& At(int idx) const
    {
        return ps[idx];
    }
    B2_FORCE_INLINE b2Support& At(int idx)
    {
        return ps[idx];
    }
    B2_FORCE_INLINE void Add(const b2Support &v)
    {
        last++;
        ps[last] = v;
    }
    B2_FORCE_INLINE void Set(int pos, const b2Support &a)
    {
        ps[pos] = a;
    }
    B2_FORCE_INLINE void SetSize(int size)
    {
        last = size - 1;
    }
    B2_FORCE_INLINE void Swap(int pos1, int pos2)
    {
        b2Support temp = ps[pos1];
        ps[pos1] = ps[pos2];
        ps[pos2] = temp;
    }
};

struct b2Polytope 
{
    struct Edge
    {
        Edge() = default;
        Edge(std::size_t i, std::size_t j) : index1(i), index2(j)
        {
        }
        B2_FORCE_INLINE void Set(std::size_t i, std::size_t j)
        {
            index1 = i;
            index2 = j;
        }
        b2Scalar dist, s;
        std::size_t index1, index2;
    };

    struct DistanceCompare
    {
        bool operator()(const Edge& lhs, const Edge& rhs) const
        {
            return lhs.dist > rhs.dist;
        }
    };

    std::priority_queue<Edge, std::vector<Edge>, DistanceCompare> edgeQueue;

    std::vector<b2Support> ps;
};

/// @brief Minkowski difference class of two shapes
struct b2MinkowskiDiff
{
    /// @brief points to two shapes
    const b2Shape* shapes[2];

    /// @brief rotation from shape0 to shape1
    b2Rot toshape1;

    /// @brief transform from shape1 to shape0 
    b2Transform toshape0;

    b2MinkowskiDiff(){}

    /// @brief b2Support function for shape0
    b2Vec2 Support0(const b2Vec2& d) const;

    /// @brief b2Support function for shape1
    b2Vec2 Support1(const b2Vec2& d) const;

    /// @brief b2Support function for the pair of shapes
    b2Vec2 Support(const b2Vec2& d) const;

    /// @brief b2Support function for the d-th shape (d = 0 or 1)
    b2Vec2 Support(const b2Vec2& d, int index) const;

    /// @brief b2Support function for the pair of shapes
    b2Support SupportS(const b2Vec2& d) const;
};

struct b2ShapeDistance 
{
    bool Separation(const b2Shape* shape1, const b2Transform &xf1,
                    const b2Shape* shape2, const b2Transform &xf2) const;

    bool Distance(const b2Shape* shape1, const b2Transform &xf1,
                  const b2Shape* shape2, const b2Transform &xf2,
                  b2Scalar *dist, b2Vec2 *p1, b2Vec2 *p2) const;

    bool SignedDistance(const b2Shape* shape1, const b2Transform &xf1,
                  const b2Shape* shape2, const b2Transform &xf2,
                  b2Scalar *dist, b2Vec2 *p1, b2Vec2 *p2) const;

    bool BisectionDistance(const b2Shape* shape1, const b2Transform &xf1,
                           const b2Shape* shape2, const b2Transform &xf2,
                           b2Scalar *dist, b2Vec2 *p1, b2Vec2 *p2) const;

    bool SignedBisectionDistance(const b2Shape* shape1, const b2Transform &xf1,
                           const b2Shape* shape2, const b2Transform &xf2,
                           b2Scalar *dist, b2Vec2 *p1, b2Vec2 *p2) const;

    /// @brief default setting for GJK algorithm
    b2ShapeDistance() = default;

    /// @brief maximum number of iterations used in GJK algorithm for distance
    int max_distance_iterations{1000};

    /// @brief the threshold used in GJK algorithm to stop distance iteration
    b2Scalar distance_tolerance{1.e-6};
};

/// @brief b2Support function for shape0
B2_FORCE_INLINE b2Vec2 b2MinkowskiDiff::Support0(const b2Vec2& d) const
{
    return shapes[0]->SupportPoint(d);
}

/// @brief b2Support function for shape1
B2_FORCE_INLINE b2Vec2 b2MinkowskiDiff::Support1(const b2Vec2& d) const
{
    return b2Mul(toshape0, shapes[1]->SupportPoint(b2Mul(toshape1, d)));
}

/// @brief b2Support function for the pair of shapes
B2_FORCE_INLINE b2Vec2 b2MinkowskiDiff::Support(const b2Vec2& d) const
{
    return Support0(d) - Support1(-d);
}

/// @brief b2Support function for the d-th shape (d = 0 or 1)
B2_FORCE_INLINE b2Vec2 b2MinkowskiDiff::Support(const b2Vec2& d, int index) const
{
    if (index)
        return Support1(d);
    else
        return Support0(d);
}

/// @brief b2Support function for the pair of shapes
B2_FORCE_INLINE b2Support b2MinkowskiDiff::SupportS(const b2Vec2& d) const
{
    b2Support s;
    s.v1 = Support0(d);
    s.v2 = Support1(-d);
    s.v = s.v1 - s.v2;
    return s;
}

#endif
