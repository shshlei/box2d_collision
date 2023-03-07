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

#include "b2_dynamic_tree.h"
#include <vector>
#include <unordered_set>

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

    /// Create a proxy with an initial AABB.
    int CreateProxy(const b2AABB& aabb, void* userData, bool active = true);

    void UpdateProxy(int proxyId, bool active);

    /// Destroy a proxy. It is up to the client to remove any pairs.
    void DestroyProxy(int proxyId);

    /// Call UpdateProxy as many times as you like, then when you are done
    void UpdateProxy(int proxyId, const b2AABB& aabb);

    /// Self collision test. For active shapes 
    template <typename T>
    bool Collide(T* callback);

    /// Self distance test. For active shapes
    template <typename T>
    bool Distance(T* callback, b2Scalar &dist);

    template <typename T>
    bool Collide(T* callback, const b2AABB& aabb) const;

    template <typename T>
    bool Collide(T* callback, const b2Vec2& point) const;

    template <typename T>
    bool Collide(T* callback, const b2BroadPhase* tree) const;

    template <typename T>
    bool Distance(T* callback, const b2AABB& aabb, b2Scalar &dist) const;

    template <typename T>
    bool Distance(T* callback, const b2Vec2& point, b2Scalar &dist) const;

    template <typename T>
    bool Distance(T* callback, const b2BroadPhase* tree, b2Scalar &dist) const;

    template <typename T>
    bool SelfDistance(T* callback, b2Scalar &dist) const;

    const b2DynamicTree* GetDynamicTree() const; 

    /// Get the fat AABB for a proxy.
    const b2AABB& GetFatAABB(int proxyId) const;

    /// Get user data from a proxy. Returns nullptr if the id is invalid.
    void* GetUserData(int proxyId) const;

    /// Test overlap of fat AABBs.
    bool TestOverlap(int proxyIdA, int proxyIdB) const;

    /// Get the number of proxies.
    int GetProxyCount() const;

    /// Get the height of the embedded tree.
    int GetTreeHeight() const;

    /// Get the balance of the embedded tree.
    int GetTreeBalance() const;

    /// Get the quality metric of the embedded tree.
    b2Scalar GetTreeQuality() const;

    /// Shift the world origin. Useful for large worlds.
    /// The shift formula is: position -= newOrigin
    /// @param newOrigin the new origin with respect to the old origin
    void ShiftOrigin(const b2Vec2& newOrigin);

    void ClearBufferActive();

    int GetQueryProxyId() const;

    bool IsActive(int proxyId) const;

    void setContactDistanceThreshold(b2Scalar contact_distance);

    b2Scalar getContactDistanceThreshold() const;

private:

    friend class b2DynamicTree;

    b2DynamicTree m_tree;

    int m_proxyCount;

    int m_queryProxyId;

    std::unordered_set<int> m_activeBuffer;
};

B2_FORCE_INLINE int b2BroadPhase::GetQueryProxyId() const
{
    return m_queryProxyId;
}

B2_FORCE_INLINE void b2BroadPhase::ClearBufferActive()
{
    m_activeBuffer.clear();
}

B2_FORCE_INLINE void* b2BroadPhase::GetUserData(int proxyId) const
{
    return m_tree.GetUserData(proxyId);
}

B2_FORCE_INLINE bool b2BroadPhase::TestOverlap(int proxyIdA, int proxyIdB) const
{
    const b2AABB& aabbA = m_tree.GetFatAABB(proxyIdA);
    const b2AABB& aabbB = m_tree.GetFatAABB(proxyIdB);
    return b2TestOverlap(aabbA, aabbB);
}

B2_FORCE_INLINE const b2AABB& b2BroadPhase::GetFatAABB(int proxyId) const
{
    return m_tree.GetFatAABB(proxyId);
}

B2_FORCE_INLINE const b2DynamicTree* b2BroadPhase::GetDynamicTree() const
{
    return &m_tree;
}

B2_FORCE_INLINE int b2BroadPhase::GetProxyCount() const
{
    return m_proxyCount;
}

B2_FORCE_INLINE int b2BroadPhase::GetTreeHeight() const
{
    return m_tree.GetHeight();
}

B2_FORCE_INLINE int b2BroadPhase::GetTreeBalance() const
{
    return m_tree.GetMaxBalance();
}

B2_FORCE_INLINE b2Scalar b2BroadPhase::GetTreeQuality() const
{
    return m_tree.GetAreaRatio();
}

template <typename T>
B2_FORCE_INLINE bool b2BroadPhase::Collide(T* callback, const b2AABB& aabb) const
{
    return m_tree.Collide(callback, aabb);
}

template <typename T>
B2_FORCE_INLINE bool b2BroadPhase::Collide(T* callback, const b2Vec2& point) const
{
    return m_tree.Collide(callback, point);
}

template <typename T>
B2_FORCE_INLINE bool b2BroadPhase::Collide(T* callback, const b2BroadPhase* tree) const
{
    return m_tree.Collide(callback, tree->GetDynamicTree());
}

template <typename T>
B2_FORCE_INLINE bool b2BroadPhase::Distance(T* callback, const b2AABB& aabb, b2Scalar &dist) const
{
    return m_tree.Distance(callback, aabb, dist);
}

template <typename T>
B2_FORCE_INLINE bool b2BroadPhase::Distance(T* callback, const b2Vec2& point, b2Scalar &dist) const
{
    return m_tree.Distance(callback, point, dist);
}

template <typename T>
B2_FORCE_INLINE bool b2BroadPhase::Distance(T* callback, const b2BroadPhase* tree, b2Scalar &dist) const
{
    return m_tree.Distance(callback, tree->GetDynamicTree(), dist);
}

template <typename T>
B2_FORCE_INLINE bool b2BroadPhase::SelfDistance(T* callback, b2Scalar &dist) const
{
    return m_tree.SelfDistance(callback, dist);
}

B2_FORCE_INLINE void b2BroadPhase::ShiftOrigin(const b2Vec2& newOrigin)
{
    m_tree.ShiftOrigin(newOrigin);
}

B2_FORCE_INLINE bool b2BroadPhase::IsActive(int proxyId) const
{
    return m_activeBuffer.find(proxyId) != m_activeBuffer.end();
}

B2_FORCE_INLINE void b2BroadPhase::setContactDistanceThreshold(b2Scalar contact_distance)
{
    m_tree.setContactDistanceThreshold(contact_distance);
}

B2_FORCE_INLINE b2Scalar b2BroadPhase::getContactDistanceThreshold() const
{
    return m_tree.getContactDistanceThreshold();
}

template <typename T>
struct b2BroadPhaseCollideWrapper
{
    // This is called from b2DynamicTree::Collide.
    bool CollideCallback(int proxyId)
    {
        // A proxy cannot form a pair with itself.
        if (proxyId == broadPhase->GetQueryProxyId())
            return false;
        if (broadPhase->IsActive(proxyId) && proxyId > broadPhase->GetQueryProxyId())
            // Both proxies are moving. Avoid duplicate pairs.
            return false;
        void* userDataA = broadPhase->GetUserData(proxyId);
        void* userDataB = broadPhase->GetUserData(broadPhase->GetQueryProxyId());
        return callback->Collide(userDataA, userDataB);
    }
    const b2BroadPhase* broadPhase{nullptr};
    T* callback{nullptr};
};

template <typename T>
bool b2BroadPhase::Collide(T* callback)
{
    b2BroadPhaseCollideWrapper<T> wrapper;
    wrapper.broadPhase = this;
    wrapper.callback = callback;
    // Perform tree queries for all moving proxies.
    for (int i : m_activeBuffer)
    {
        m_queryProxyId = i;
        if (m_queryProxyId == e_nullProxy)
            continue;
        // We have to query the tree with the fat AABB so that
        // we don't fail to create a pair that may touch later.
        const b2AABB& fatAABB = m_tree.GetFatAABB(m_queryProxyId);
        if (m_tree.Collide(&wrapper, fatAABB))
            return true;
    }
    return false;
}

template <typename T>
struct b2BroadPhaseDistanceWrapper
{
    // This is called from b2DynamicTree::Distance.
    bool DistanceCallback(int proxyId, b2Scalar &dist)
    {
        // A proxy cannot form a pair with itself.
        if (proxyId == broadPhase->GetQueryProxyId())
            return false;
        if (broadPhase->IsActive(proxyId) && proxyId > broadPhase->GetQueryProxyId())
            // Both proxies are moving. Avoid duplicate pairs.
            return false;
        void* userDataA = broadPhase->GetUserData(proxyId);
        void* userDataB = broadPhase->GetUserData(broadPhase->GetQueryProxyId());
        return callback->Distance(userDataA, userDataB, dist);
    }
    const b2BroadPhase* broadPhase{nullptr};
    T* callback{nullptr};
};

template <typename T>
bool b2BroadPhase::Distance(T* callback, b2Scalar &dist)
{
    b2BroadPhaseDistanceWrapper<T> wrapper;
    wrapper.broadPhase = this;
    wrapper.callback = callback;
    // Perform tree queries for all moving proxies.
    for (int i : m_activeBuffer)
    {
        m_queryProxyId = i;
        if (m_queryProxyId == e_nullProxy)
            continue;
        // We have to query the tree with the fat AABB so that
        // we don't fail to create a pair that may touch later.
        const b2AABB& fatAABB = m_tree.GetFatAABB(m_queryProxyId);
        if (!m_tree.Distance(&wrapper, fatAABB, dist))
            return false;
    }
    return true;
}

#endif
