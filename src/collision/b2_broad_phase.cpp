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

int b2BroadPhase::CreateProxy(const b2AABB& aabb, void* userData, bool active)
{
    int proxyId = m_tree.CreateProxy(aabb, userData);
    ++m_proxyCount;
    if (active)
        m_activeBuffer.insert(proxyId);
    return proxyId;
}

void b2BroadPhase::UpdateProxy(int proxyId, bool active)
{
    if (active)
        m_activeBuffer.insert(proxyId);
    else
        m_activeBuffer.erase(proxyId);
}

void b2BroadPhase::DestroyProxy(int proxyId)
{
    m_activeBuffer.erase(proxyId);
    --m_proxyCount;
    m_tree.DestroyProxy(proxyId);
}

void b2BroadPhase::UpdateProxy(int proxyId, const b2AABB& aabb)
{
    m_tree.UpdateProxy(proxyId, aabb);
}
