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

#ifndef B2_BROAD_PHASE_H
#define B2_BROAD_PHASE_H

#include "b2_api.h"
#include "b2_settings.h"
#include "b2_collision.h"
#include "b2_dynamic_tree.h"

struct B2_API b2Pair
{
    int32 proxyIdA;
    int32 proxyIdB;
};

/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
class B2_API b2BroadPhase
{
public:

    enum
    {
        e_nullProxy = -1
    };

    b2BroadPhase();
    ~b2BroadPhase();

    /// Create a proxy with an initial AABB. Pairs are not reported until
    /// UpdatePairs is called.
    int32 CreateProxy(const b2AABB& aabb, void* userData, bool extend = true);

    void UpdateProxy(int32 proxyId, bool extend);

    /// Destroy a proxy. It is up to the client to remove any pairs.
    void DestroyProxy(int32 proxyId);

    /// Call MoveProxy as many times as you like, then when you are done
    /// call UpdatePairs to finalized the proxy pairs (for your time step).
    void MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement);

    /// Call UpdateProxy as many times as you like, then when you are done
    /// call UpdatePairs to finalized the proxy pairs (for your time step).
    void UpdateProxy(int32 proxyId, const b2AABB& aabb);

    /// Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
    void TouchProxy(int32 proxyId);

    /// Get the fat AABB for a proxy.
    const b2AABB& GetFatAABB(int32 proxyId) const;

    /// Get user data from a proxy. Returns nullptr if the id is invalid.
    void* GetUserData(int32 proxyId) const;

    /// Test overlap of fat AABBs.
    bool TestOverlap(int32 proxyIdA, int32 proxyIdB) const;

    /// Get the number of proxies.
    int32 GetProxyCount() const;

    /// Update the pairs. This results in pair callbacks. This can only add pairs.
    template <typename T>
    void UpdatePairs(T* callback);

    /// Query an AABB for overlapping proxies. The callback class
    /// is called for each proxy that overlaps the supplied AABB.
    template <typename T>
    void Query(T* callback, const b2AABB& aabb) const;

    /// Query a point for overlapping proxies. The callback class
    /// is called for each proxy that contains the supplied point.
    template <typename T>
    void Query(T* callback, const b2Vec2& point) const;

    template <typename T>
    void Query(T* callback, const b2BroadPhase* tree) const;

    /// Ray-cast against the proxies in the tree. This relies on the callback
    /// to perform a exact ray-cast in the case were the proxy contains a shape.
    /// The callback also performs the any collision filtering. This has performance
    /// roughly equal to k * log(n), where k is the number of collisions and n is the
    /// number of proxies in the tree.
    /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
    /// @param callback a callback class that is called for each proxy that is hit by the ray.
    template <typename T>
    void RayCast(T* callback, const b2RayCastInput& input) const;

    const b2DynamicTree* GetDynamicTree() const; 

    /// Get the height of the embedded tree.
    int32 GetTreeHeight() const;

    /// Get the balance of the embedded tree.
    int32 GetTreeBalance() const;

    /// Get the quality metric of the embedded tree.
    b2Scalar GetTreeQuality() const;

    /// Shift the world origin. Useful for large worlds.
    /// The shift formula is: position -= newOrigin
    /// @param newOrigin the new origin with respect to the old origin
    void ShiftOrigin(const b2Vec2& newOrigin);

    void ClearBufferMove();

    void ClearPairs();

private:

    friend class b2DynamicTree;

    void BufferMove(int32 proxyId);
    void UnBufferMove(int32 proxyId);

    bool QueryCallback(int32 proxyId);

    b2DynamicTree m_tree;

    int32 m_proxyCount;

    int32* m_moveBuffer;
    int32 m_moveCapacity;
    int32 m_moveCount;

    b2Pair* m_pairBuffer;
    int32 m_pairCapacity;
    int32 m_pairCount;

    int32 m_queryProxyId;
};

B2_FORCE_INLINE void b2BroadPhase::ClearBufferMove()
{
    m_moveCount = 0;
}

B2_FORCE_INLINE void b2BroadPhase::ClearPairs()
{
    m_pairCount = 0;
}

B2_FORCE_INLINE void* b2BroadPhase::GetUserData(int32 proxyId) const
{
    return m_tree.GetUserData(proxyId);
}

B2_FORCE_INLINE bool b2BroadPhase::TestOverlap(int32 proxyIdA, int32 proxyIdB) const
{
    const b2AABB& aabbA = m_tree.GetFatAABB(proxyIdA);
    const b2AABB& aabbB = m_tree.GetFatAABB(proxyIdB);
    return b2TestOverlap(aabbA, aabbB);
}

B2_FORCE_INLINE const b2AABB& b2BroadPhase::GetFatAABB(int32 proxyId) const
{
    return m_tree.GetFatAABB(proxyId);
}

B2_FORCE_INLINE const b2DynamicTree* b2BroadPhase::GetDynamicTree() const
{
    return &m_tree;
}

B2_FORCE_INLINE int32 b2BroadPhase::GetProxyCount() const
{
    return m_proxyCount;
}

B2_FORCE_INLINE int32 b2BroadPhase::GetTreeHeight() const
{
    return m_tree.GetHeight();
}

B2_FORCE_INLINE int32 b2BroadPhase::GetTreeBalance() const
{
    return m_tree.GetMaxBalance();
}

B2_FORCE_INLINE b2Scalar b2BroadPhase::GetTreeQuality() const
{
    return m_tree.GetAreaRatio();
}

template <typename T>
void b2BroadPhase::UpdatePairs(T* callback)
{
    // Reset pair buffer
    m_pairCount = 0;

    // Perform tree queries for all moving proxies.
    for (int32 i = 0; i < m_moveCount; ++i)
    {
        m_queryProxyId = m_moveBuffer[i];
        if (m_queryProxyId == e_nullProxy)
        {
            continue;
        }

        // We have to query the tree with the fat AABB so that
        // we don't fail to create a pair that may touch later.
        const b2AABB& fatAABB = m_tree.GetFatAABB(m_queryProxyId);

        // Query tree, create pairs and add them pair buffer.
        m_tree.Query(this, fatAABB);
    }

    // Send pairs to caller
    for (int32 i = 0; i < m_pairCount; ++i)
    {
        b2Pair* primaryPair = m_pairBuffer + i;
        void* userDataA = m_tree.GetUserData(primaryPair->proxyIdA);
        void* userDataB = m_tree.GetUserData(primaryPair->proxyIdB);

        callback->AddPair(userDataA, userDataB);
    }

    // Clear move flags
    for (int32 i = 0; i < m_moveCount; ++i)
    {
        int32 proxyId = m_moveBuffer[i];
        if (proxyId == e_nullProxy)
        {
            continue;
        }

        m_tree.ClearMoved(proxyId);
    }

    // Reset move buffer
    m_moveCount = 0;
}

template <typename T>
B2_FORCE_INLINE void b2BroadPhase::Query(T* callback, const b2AABB& aabb) const
{
    m_tree.Query(callback, aabb);
}

template <typename T>
B2_FORCE_INLINE void b2BroadPhase::Query(T* callback, const b2Vec2& aabb) const
{
    m_tree.Query(callback, aabb);
}

template <typename T>
B2_FORCE_INLINE void b2BroadPhase::Query(T* callback, const b2BroadPhase* tree) const
{
    m_tree.Query(callback, tree->GetDynamicTree());
}

template <typename T>
B2_FORCE_INLINE void b2BroadPhase::RayCast(T* callback, const b2RayCastInput& input) const
{
    m_tree.RayCast(callback, input);
}

B2_FORCE_INLINE void b2BroadPhase::ShiftOrigin(const b2Vec2& newOrigin)
{
    m_tree.ShiftOrigin(newOrigin);
}

#endif
