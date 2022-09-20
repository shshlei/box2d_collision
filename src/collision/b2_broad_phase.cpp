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

#include "box2d_collision/b2_broad_phase.h"

b2BroadPhase::b2BroadPhase()
{
    m_proxyCount = 0;
}

b2BroadPhase::~b2BroadPhase()
{
}

int32 b2BroadPhase::CreateProxy(const b2AABB& aabb, void* userData, bool extend)
{
    int32 proxyId = m_tree.CreateProxy(aabb, userData, extend);
    ++m_proxyCount;
    if (extend)
    {
        m_moveBuffer.insert(proxyId);
        m_activeBuffer.insert(proxyId);
    }
    return proxyId;
}

void b2BroadPhase::UpdateProxy(int32 proxyId, bool extend)
{
    if (extend)
    {
        if (m_tree.UpdateProxy(proxyId, extend))
            m_moveBuffer.insert(proxyId);
        m_activeBuffer.insert(proxyId);
    }
    else
    {
        if (m_tree.UpdateProxy(proxyId, extend))
            m_moveBuffer.erase(proxyId);
        m_activeBuffer.erase(proxyId);
    }
}

void b2BroadPhase::DestroyProxy(int32 proxyId)
{
    m_moveBuffer.erase(proxyId);
    m_activeBuffer.erase(proxyId);
    --m_proxyCount;
    m_tree.DestroyProxy(proxyId);
}

void b2BroadPhase::MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement)
{
    if (m_tree.MoveProxy(proxyId, aabb, displacement))
        m_moveBuffer.insert(proxyId);
}

void b2BroadPhase::UpdateProxy(int32 proxyId, const b2AABB& aabb)
{
    if (m_tree.UpdateProxy(proxyId, aabb))
        m_moveBuffer.insert(proxyId);
}

void b2BroadPhase::TouchProxy(int32 proxyId)
{
    m_moveBuffer.insert(proxyId);
}

// This is called from b2DynamicTree::Query when we are gathering pairs.
bool b2BroadPhase::QueryCallback(int32 proxyId)
{
    // A proxy cannot form a pair with itself.
    if (proxyId == m_queryProxyId)
        return true;

    if (WasMoved(proxyId) && proxyId > m_queryProxyId)
        // Both proxies are moving. Avoid duplicate pairs.
        return true;
    m_pairBuffer.emplace_back(b2Min(proxyId, m_queryProxyId), b2Max(proxyId, m_queryProxyId));
    return true;
}

// This is called from b2DynamicTree::Query when we are gathering pairs.
bool b2BroadPhase::QueryCallbackDistance(int32 proxyId)
{
    // A proxy cannot form a pair with itself.
    if (proxyId == m_queryProxyId)
        return true;

    if (IsActive(proxyId) && proxyId > m_queryProxyId)
        // Both proxies are moving. Avoid duplicate pairs.
        return true;
    m_pairBuffer.emplace_back(b2Min(proxyId, m_queryProxyId), b2Max(proxyId, m_queryProxyId));
    return true;
}
