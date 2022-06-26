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
#include "box2d_collision/b2_distance.h"
#include "box2d_collision/b2_circle_shape.h"
#include "box2d_collision/b2_polygon_shape.h"
#include "box2d_collision/b2_time_of_impact.h"
#include "box2d_collision/b2_timer.h"

#include <stdio.h>

B2_API b2Scalar b2_toiTime, b2_toiMaxTime;
B2_API int32 b2_toiCalls, b2_toiIters, b2_toiMaxIters;
B2_API int32 b2_toiRootIters, b2_toiMaxRootIters;

//
struct b2SeparationFunction
{
    enum Type
    {
        e_points,
        e_faceA,
        e_faceB
    };

    // TODO_ERIN might not need to return the separation

    b2Scalar Initialize(const b2SimplexCache* cache,
            const b2DistanceProxy* proxyA, const b2Sweep& sweepA,
            const b2DistanceProxy* proxyB, const b2Sweep& sweepB,
            b2Scalar t1)
    {
        m_proxyA = proxyA;
        m_proxyB = proxyB;
        int32 count = cache->count;
//        b2Assert(0 < count && count < 3);

        m_sweepA = sweepA;
        m_sweepB = sweepB;

        b2Transform xfA, xfB;
        m_sweepA.GetTransform(&xfA, t1);
        m_sweepB.GetTransform(&xfB, t1);

        if (count == 1)
        {
            m_type = e_points;
            b2Vec2 localPointA = m_proxyA->GetVertex(cache->indexA[0]);
            b2Vec2 localPointB = m_proxyB->GetVertex(cache->indexB[0]);
            b2Vec2 pointA = b2Mul(xfA, localPointA);
            b2Vec2 pointB = b2Mul(xfB, localPointB);
            m_axis = pointB - pointA;
            b2Scalar s = m_axis.Normalize();
            return s;
        }
        else if (cache->indexA[0] == cache->indexA[1])
        {
            // Two points on B and one on A.
            m_type = e_faceB;
            b2Vec2 localPointB1 = proxyB->GetVertex(cache->indexB[0]);
            b2Vec2 localPointB2 = proxyB->GetVertex(cache->indexB[1]);

            m_axis = b2Cross(localPointB2 - localPointB1, b2Scalar(1.0));
            m_axis.Normalize();
            b2Vec2 normal = b2Mul(xfB.q, m_axis);

            m_localPoint = b2Scalar(0.5) * (localPointB1 + localPointB2);
            b2Vec2 pointB = b2Mul(xfB, m_localPoint);

            b2Vec2 localPointA = proxyA->GetVertex(cache->indexA[0]);
            b2Vec2 pointA = b2Mul(xfA, localPointA);

            b2Scalar s = b2Dot(pointA - pointB, normal);
            if (s < b2Scalar(0.0))
            {
                m_axis = -m_axis;
                s = -s;
            }
            return s;
        }
        else
        {
            // Two points on A and one or two points on B.
            m_type = e_faceA;
            b2Vec2 localPointA1 = m_proxyA->GetVertex(cache->indexA[0]);
            b2Vec2 localPointA2 = m_proxyA->GetVertex(cache->indexA[1]);

            m_axis = b2Cross(localPointA2 - localPointA1, b2Scalar(1.0));
            m_axis.Normalize();
            b2Vec2 normal = b2Mul(xfA.q, m_axis);

            m_localPoint = b2Scalar(0.5) * (localPointA1 + localPointA2);
            b2Vec2 pointA = b2Mul(xfA, m_localPoint);

            b2Vec2 localPointB = m_proxyB->GetVertex(cache->indexB[0]);
            b2Vec2 pointB = b2Mul(xfB, localPointB);

            b2Scalar s = b2Dot(pointB - pointA, normal);
            if (s < b2Scalar(0.0))
            {
                m_axis = -m_axis;
                s = -s;
            }
            return s;
        }
    }

    //
    b2Scalar FindMinSeparation(int32* indexA, int32* indexB, b2Scalar t) const
    {
        b2Transform xfA, xfB;
        m_sweepA.GetTransform(&xfA, t);
        m_sweepB.GetTransform(&xfB, t);

        switch (m_type)
        {
            case e_points:
                {
                    b2Vec2 axisA = b2MulT(xfA.q,  m_axis);
                    b2Vec2 axisB = b2MulT(xfB.q, -m_axis);

                    *indexA = m_proxyA->GetSupport(axisA);
                    *indexB = m_proxyB->GetSupport(axisB);

                    b2Vec2 localPointA = m_proxyA->GetVertex(*indexA);
                    b2Vec2 localPointB = m_proxyB->GetVertex(*indexB);

                    b2Vec2 pointA = b2Mul(xfA, localPointA);
                    b2Vec2 pointB = b2Mul(xfB, localPointB);

                    b2Scalar separation = b2Dot(pointB - pointA, m_axis);
                    return separation;
                }

            case e_faceA:
                {
                    b2Vec2 normal = b2Mul(xfA.q, m_axis);
                    b2Vec2 pointA = b2Mul(xfA, m_localPoint);

                    b2Vec2 axisB = b2MulT(xfB.q, -normal);

                    *indexA = -1;
                    *indexB = m_proxyB->GetSupport(axisB);

                    b2Vec2 localPointB = m_proxyB->GetVertex(*indexB);
                    b2Vec2 pointB = b2Mul(xfB, localPointB);

                    b2Scalar separation = b2Dot(pointB - pointA, normal);
                    return separation;
                }

            case e_faceB:
                {
                    b2Vec2 normal = b2Mul(xfB.q, m_axis);
                    b2Vec2 pointB = b2Mul(xfB, m_localPoint);

                    b2Vec2 axisA = b2MulT(xfA.q, -normal);

                    *indexB = -1;
                    *indexA = m_proxyA->GetSupport(axisA);

                    b2Vec2 localPointA = m_proxyA->GetVertex(*indexA);
                    b2Vec2 pointA = b2Mul(xfA, localPointA);

                    b2Scalar separation = b2Dot(pointA - pointB, normal);
                    return separation;
                }

            default:
                b2Assert(false);
                *indexA = -1;
                *indexB = -1;
                return b2Scalar(0.0);
        }
    }

    //
    b2Scalar Evaluate(int32 indexA, int32 indexB, b2Scalar t) const
    {
        b2Transform xfA, xfB;
        m_sweepA.GetTransform(&xfA, t);
        m_sweepB.GetTransform(&xfB, t);

        switch (m_type)
        {
            case e_points:
                {
                    b2Vec2 localPointA = m_proxyA->GetVertex(indexA);
                    b2Vec2 localPointB = m_proxyB->GetVertex(indexB);

                    b2Vec2 pointA = b2Mul(xfA, localPointA);
                    b2Vec2 pointB = b2Mul(xfB, localPointB);
                    b2Scalar separation = b2Dot(pointB - pointA, m_axis);

                    return separation;
                }

            case e_faceA:
                {
                    b2Vec2 normal = b2Mul(xfA.q, m_axis);
                    b2Vec2 pointA = b2Mul(xfA, m_localPoint);

                    b2Vec2 localPointB = m_proxyB->GetVertex(indexB);
                    b2Vec2 pointB = b2Mul(xfB, localPointB);

                    b2Scalar separation = b2Dot(pointB - pointA, normal);
                    return separation;
                }

            case e_faceB:
                {
                    b2Vec2 normal = b2Mul(xfB.q, m_axis);
                    b2Vec2 pointB = b2Mul(xfB, m_localPoint);

                    b2Vec2 localPointA = m_proxyA->GetVertex(indexA);
                    b2Vec2 pointA = b2Mul(xfA, localPointA);

                    b2Scalar separation = b2Dot(pointA - pointB, normal);
                    return separation;
                }

            default:
                b2Assert(false);
                return b2Scalar(0.0);
        }
    }

    const b2DistanceProxy* m_proxyA;
    const b2DistanceProxy* m_proxyB;
    b2Sweep m_sweepA, m_sweepB;
    Type m_type;
    b2Vec2 m_localPoint;
    b2Vec2 m_axis;
};

// CCD via the local separating axis method. This seeks progression
// by computing the largest time at which separation is maintained.
void b2TimeOfImpact(b2TOIOutput* output, const b2TOIInput* input)
{
    b2Timer timer;

    ++b2_toiCalls;

    output->state = b2TOIOutput::e_unknown;
    output->t = input->tMax;

    const b2DistanceProxy* proxyA = &input->proxyA;
    const b2DistanceProxy* proxyB = &input->proxyB;

    b2Sweep sweepA = input->sweepA;
    b2Sweep sweepB = input->sweepB;

    // Large rotations can make the root finder fail, so we normalize the
    // sweep angles.
    sweepA.Normalize();
    sweepB.Normalize();

    b2Scalar tMax = input->tMax;

    b2Scalar totalRadius = proxyA->m_radius + proxyB->m_radius;
    b2Scalar target = b2Max(b2_linearSlop, totalRadius - b2Scalar(3.0) * b2_linearSlop);
    b2Scalar tolerance = b2Scalar(0.25) * b2_linearSlop;
//    b2Assert(target > tolerance);

    b2Scalar t1 = b2Scalar(0.0);
    const int32 k_maxIterations = 20;	// TODO_ERIN b2Settings
    int32 iter = 0;

    // Prepare input for distance query.
    b2SimplexCache cache;
    cache.count = 0;
    b2DistanceInput distanceInput;
    distanceInput.proxyA = input->proxyA;
    distanceInput.proxyB = input->proxyB;
    distanceInput.useRadii = false;

    // The outer loop progressively attempts to compute new separating axes.
    // This loop terminates when an axis is repeated (no progress is made).
    for(;;)
    {
        b2Transform xfA, xfB;
        sweepA.GetTransform(&xfA, t1);
        sweepB.GetTransform(&xfB, t1);

        // Get the distance between shapes. We can also use the results
        // to get a separating axis.
        distanceInput.transformA = xfA;
        distanceInput.transformB = xfB;
        b2DistanceOutput distanceOutput;
        b2Distance(&distanceOutput, &cache, &distanceInput);

        // If the shapes are overlapped, we give up on continuous collision.
        if (distanceOutput.distance <= b2Scalar(0.0))
        {
            // Failure!
            output->state = b2TOIOutput::e_overlapped;
            output->t = b2Scalar(0.0);
            break;
        }

        if (distanceOutput.distance < target + tolerance)
        {
            // Victory!
            output->state = b2TOIOutput::e_touching;
            output->t = t1;
            break;
        }

        // Initialize the separating axis.
        b2SeparationFunction fcn;
        fcn.Initialize(&cache, proxyA, sweepA, proxyB, sweepB, t1);
#if 0
        // Dump the curve seen by the root finder
        {
            const int32 N = 100;
            b2Scalar dx = b2Scalar(1.0) / N;
            b2Scalar xs[N+1];
            b2Scalar fs[N+1];

            b2Scalar x = b2Scalar(0.0);

            for (int32 i = 0; i <= N; ++i)
            {
                sweepA.GetTransform(&xfA, x);
                sweepB.GetTransform(&xfB, x);
                b2Scalar f = fcn.Evaluate(xfA, xfB) - target;

                printf("%g %g\n", x, f);

                xs[i] = x;
                fs[i] = f;

                x += dx;
            }
        }
#endif

        // Compute the TOI on the separating axis. We do this by successively
        // resolving the deepest point. This loop is bounded by the number of vertices.
        bool done = false;
        b2Scalar t2 = tMax;
        int32 pushBackIter = 0;
        for (;;)
        {
            // Find the deepest point at t2. Store the witness point indices.
            int32 indexA, indexB;
            b2Scalar s2 = fcn.FindMinSeparation(&indexA, &indexB, t2);

            // Is the final configuration separated?
            if (s2 > target + tolerance)
            {
                // Victory!
                output->state = b2TOIOutput::e_separated;
                output->t = tMax;
                done = true;
                break;
            }

            // Has the separation reached tolerance?
            if (s2 > target - tolerance)
            {
                // Advance the sweeps
                t1 = t2;
                break;
            }

            // Compute the initial separation of the witness points.
            b2Scalar s1 = fcn.Evaluate(indexA, indexB, t1);

            // Check for initial overlap. This might happen if the root finder
            // runs out of iterations.
            if (s1 < target - tolerance)
            {
                output->state = b2TOIOutput::e_failed;
                output->t = t1;
                done = true;
                break;
            }

            // Check for touching
            if (s1 <= target + tolerance)
            {
                // Victory! t1 should hold the TOI (could be 0.0).
                output->state = b2TOIOutput::e_touching;
                output->t = t1;
                done = true;
                break;
            }

            // Compute 1D root of: f(x) - target = 0
            int32 rootIterCount = 0;
            b2Scalar a1 = t1, a2 = t2;
            for (;;)
            {
                // Use a mix of the secant rule and bisection.
                b2Scalar t;
                if (rootIterCount & 1)
                {
                    // Secant rule to improve convergence.
                    t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
                }
                else
                {
                    // Bisection to guarantee progress.
                    t = b2Scalar(0.5) * (a1 + a2);
                }

                ++rootIterCount;
                ++b2_toiRootIters;

                b2Scalar s = fcn.Evaluate(indexA, indexB, t);

                if (b2Abs(s - target) < tolerance)
                {
                    // t2 holds a tentative value for t1
                    t2 = t;
                    break;
                }

                // Ensure we continue to bracket the root.
                if (s > target)
                {
                    a1 = t;
                    s1 = s;
                }
                else
                {
                    a2 = t;
                    s2 = s;
                }

                if (rootIterCount == 50)
                {
                    break;
                }
            }

            b2_toiMaxRootIters = b2Max(b2_toiMaxRootIters, rootIterCount);

            ++pushBackIter;

            if (pushBackIter == b2_maxPolygonVertices)
            {
                break;
            }
        }

        ++iter;
        ++b2_toiIters;

        if (done)
        {
            break;
        }

        if (iter == k_maxIterations)
        {
            // Root finder got stuck. Semi-victory.
            output->state = b2TOIOutput::e_failed;
            output->t = t1;
            break;
        }
    }

    b2_toiMaxIters = b2Max(b2_toiMaxIters, iter);

    b2Scalar time = timer.GetMilliseconds();
    b2_toiMaxTime = b2Max(b2_toiMaxTime, time);
    b2_toiTime += time;
}
