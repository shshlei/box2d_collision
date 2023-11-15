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

int b2BroadPhase::GetQueryProxyId() const
{
  return m_queryProxyId;
}

void b2BroadPhase::ClearBufferActive()
{
  m_activeBuffer.clear();
}

void * b2BroadPhase::GetUserData(int proxyId) const
{
  return m_tree.GetUserData(proxyId);
}

bool b2BroadPhase::TestOverlap(int proxyIdA, int proxyIdB) const
{
  const b2AABB & aabbA = m_tree.GetFatAABB(proxyIdA);
  const b2AABB & aabbB = m_tree.GetFatAABB(proxyIdB);
  return b2TestOverlap(aabbA, aabbB);
}

const b2AABB & b2BroadPhase::GetFatAABB(int proxyId) const
{
  return m_tree.GetFatAABB(proxyId);
}

const b2DynamicTree * b2BroadPhase::GetDynamicTree() const
{
  return &m_tree;
}

int b2BroadPhase::GetProxyCount() const
{
  return m_proxyCount;
}

int b2BroadPhase::GetTreeHeight() const
{
  return m_tree.GetHeight();
}

int b2BroadPhase::GetTreeBalance() const
{
  return m_tree.GetMaxBalance();
}

b2Scalar b2BroadPhase::GetTreeQuality() const
{
  return m_tree.GetAreaRatio();
}

void b2BroadPhase::ShiftOrigin(const b2Vec2 & newOrigin)
{
  m_tree.ShiftOrigin(newOrigin);
}

bool b2BroadPhase::IsActive(int proxyId) const
{
  return m_activeBuffer.find(proxyId) != m_activeBuffer.end();
}

void b2BroadPhase::setContactDistanceThreshold(b2Scalar contact_distance)
{
  m_tree.setContactDistanceThreshold(contact_distance);
}

b2Scalar b2BroadPhase::getContactDistanceThreshold() const
{
  return m_tree.getContactDistanceThreshold();
}

int b2BroadPhase::CreateProxy(const b2AABB & aabb, void * userData, bool active)
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

void b2BroadPhase::UpdateProxy(int proxyId, const b2AABB & aabb)
{
  m_tree.UpdateProxy(proxyId, aabb);
}
