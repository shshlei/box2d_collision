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

#include "box2d_collision/b2_fixture.h"

#include "box2d_collision/b2_block_allocator.h"
#include "box2d_collision/b2_broad_phase.h"
#include "box2d_collision/b2_shape_capsule.h"
#include "box2d_collision/b2_shape_circle.h"
#include "box2d_collision/b2_shape_ellipse.h"
#include "box2d_collision/b2_shape_polygon.h"
#include "box2d_collision/b2_shape_rectangle.h"

b2Fixture::b2Fixture()
{
  m_body = nullptr;
  m_next = nullptr;
  m_proxies = nullptr;
  m_shape = nullptr;
}

b2Shape::Type b2Fixture::GetType() const
{
  return m_shape->GetType();
}

b2Shape * b2Fixture::GetShape()
{
  return m_shape;
}

const b2Shape * b2Fixture::GetShape() const
{
  return m_shape;
}

void b2Fixture::SetFilterData(const b2Filter & filter)
{
  m_filter = filter;
}

const b2Filter & b2Fixture::GetFilterData() const
{
  return m_filter;
}

void b2Fixture::SetUserData(const b2FixtureUserData & userData)
{
  m_userData = userData;
}

b2FixtureUserData & b2Fixture::GetUserData()
{
  return m_userData;
}

const b2FixtureUserData & b2Fixture::GetUserData() const
{
  return m_userData;
}

b2Body * b2Fixture::GetBody()
{
  return m_body;
}

const b2Body * b2Fixture::GetBody() const
{
  return m_body;
}

b2Fixture * b2Fixture::GetNext()
{
  return m_next;
}

const b2Fixture * b2Fixture::GetNext() const
{
  return m_next;
}

bool b2Fixture::TestPoint(const b2Vec2 & p) const
{
  if (m_proxies->fixture)
    return m_shape->TestPoint(m_proxies->xf, p);
  if (m_identity)
    return m_shape->TestPoint(m_body->GetTransform(), p);
  return m_shape->TestPoint(b2Mul(m_body->GetTransform(), m_xf), p);
}

const b2AABB & b2Fixture::GetAABB() const
{
  if (m_proxies->fixture)
    return m_proxies->aabb;
  const b2Transform & xf = m_body->GetTransform();
  if (!m_identity)
    m_shape->ComputeAABB(&m_proxies->aabb, b2Mul(xf, m_xf));
  else
    m_shape->ComputeAABB(&m_proxies->aabb, xf);
  return m_proxies->aabb;
}

int b2Fixture::GetProxyId() const
{
  return m_proxies->proxyId;
}

const b2Transform & b2Fixture::GetLocalTransform() const
{
  return m_xf;
}

const b2Transform & b2Fixture::GetGlobalTransform() const
{
  return m_proxies->xf;
}

void b2Fixture::Create(b2BlockAllocator * allocator, b2Body * body, const b2FixtureDef * def)
{
  m_userData = def->userData;

  m_body = body;
  m_next = nullptr;

  m_filter = def->filter;

  m_shape = def->shape->Clone(allocator);

  m_xf = def->xf;
  m_identity = m_xf.isApprox(b2Transform::Identity());

  // Reserve proxy space
  m_proxies = (b2FixtureProxy *)allocator->Allocate(sizeof(b2FixtureProxy));
  m_proxies->fixture = nullptr;
  m_proxies->proxyId = b2BroadPhase::e_nullProxy;
}

void b2Fixture::Destroy(b2BlockAllocator * allocator)
{
  // Free the proxy array.
  allocator->Free(m_proxies, sizeof(b2FixtureProxy));
  m_proxies = nullptr;

  // Free the child shape.
  switch (m_shape->GetType()) {
    case b2Shape::e_circle:
    {
      b2CircleShape * s = (b2CircleShape *)m_shape;
      s->~b2CircleShape();
      allocator->Free(s, sizeof(b2CircleShape));
    } break;

    case b2Shape::e_ellipse:
    {
      b2EllipseShape * s = (b2EllipseShape *)m_shape;
      s->~b2EllipseShape();
      allocator->Free(s, sizeof(b2EllipseShape));
    } break;

    case b2Shape::e_capsule:
    {
      b2CapsuleShape * s = (b2CapsuleShape *)m_shape;
      s->~b2CapsuleShape();
      allocator->Free(s, sizeof(b2CapsuleShape));
    } break;

    case b2Shape::e_rectangle:
    {
      b2RectangleShape * s = (b2RectangleShape *)m_shape;
      s->~b2RectangleShape();
      allocator->Free(s, sizeof(b2RectangleShape));
    } break;

    case b2Shape::e_polygon:
    {
      b2PolygonShape * s = (b2PolygonShape *)m_shape;
      s->~b2PolygonShape();
      allocator->Free(s, sizeof(b2PolygonShape));
    } break;

    default:
      assert(false);
      break;
  }

  m_shape = nullptr;
}

void b2Fixture::CreateProxies(b2BroadPhase * broadPhase, const b2Transform & xf, bool active)
{
  b2FixtureProxy * proxy = m_proxies;
  proxy->xf = xf;
  if (!m_identity)
    proxy->xf = b2Mul(xf, m_xf);
  m_shape->ComputeAABB(&proxy->aabb, proxy->xf);
  proxy->proxyId = broadPhase->CreateProxy(proxy->aabb, proxy, active);
  proxy->fixture = this;
}

void b2Fixture::UpdateProxies(b2BroadPhase * broadPhase, bool active)
{
  b2FixtureProxy * proxy = m_proxies;
  broadPhase->UpdateProxy(proxy->proxyId, active);
}

void b2Fixture::DestroyProxies(b2BroadPhase * broadPhase)
{
  // Destroy proxies in the broad-phase.
  b2FixtureProxy * proxy = m_proxies;
  broadPhase->DestroyProxy(proxy->proxyId);
  proxy->proxyId = b2BroadPhase::e_nullProxy;
}

void b2Fixture::Update(b2BroadPhase * broadPhase, const b2Transform & xf)
{
  b2FixtureProxy * proxy = m_proxies;
  proxy->xf = xf;
  if (!m_identity)
    proxy->xf = b2Mul(xf, m_xf);
  m_shape->ComputeAABB(&proxy->aabb, proxy->xf);
  broadPhase->UpdateProxy(proxy->proxyId, proxy->aabb);
}
