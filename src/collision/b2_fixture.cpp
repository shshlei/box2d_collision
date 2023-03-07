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
#include "box2d_collision/b2_shape_circle.h"
#include "box2d_collision/b2_shape_ellipse.h"
#include "box2d_collision/b2_shape_capsule.h"
#include "box2d_collision/b2_shape_rectangle.h"
#include "box2d_collision/b2_shape_polygon.h"
#include "box2d_collision/b2_collision.h"

b2Fixture::b2Fixture()
{
    m_body = nullptr;
    m_next = nullptr;
    m_proxies = nullptr;
    m_shape = nullptr;
}

void b2Fixture::Create(b2BlockAllocator* allocator, b2Body* body, const b2FixtureDef* def)
{
    m_userData = def->userData;

    m_body = body;
    m_next = nullptr;

    m_filter = def->filter;

    m_shape = def->shape->Clone(allocator);

    m_xf = def->xf;
    m_identity = m_xf.IsIdentity();

    // Reserve proxy space
    m_proxies = (b2FixtureProxy*)allocator->Allocate(sizeof(b2FixtureProxy));
    m_proxies->fixture = nullptr;
    m_proxies->proxyId = b2BroadPhase::e_nullProxy;
}

void b2Fixture::Destroy(b2BlockAllocator* allocator)
{
    // Free the proxy array.
    allocator->Free(m_proxies, sizeof(b2FixtureProxy));
    m_proxies = nullptr;

    // Free the child shape.
    switch (m_shape->m_type)
    {
        case b2Shape::e_circle:
        {
            b2CircleShape* s = (b2CircleShape*)m_shape;
            s->~b2CircleShape();
            allocator->Free(s, sizeof(b2CircleShape));
        }
        break;

        case b2Shape::e_ellipse:
        {
            b2EllipseShape* s = (b2EllipseShape*)m_shape;
            s->~b2EllipseShape();
            allocator->Free(s, sizeof(b2EllipseShape));
        }
        break;

        case b2Shape::e_capsule:
        {
            b2CapsuleShape* s = (b2CapsuleShape*)m_shape;
            s->~b2CapsuleShape();
            allocator->Free(s, sizeof(b2CapsuleShape));
        }
        break;

        case b2Shape::e_rectangle:
        {
            b2RectangleShape* s = (b2RectangleShape*)m_shape;
            s->~b2RectangleShape();
            allocator->Free(s, sizeof(b2RectangleShape));
        }
        break;

        case b2Shape::e_polygon:
        {
            b2PolygonShape* s = (b2PolygonShape*)m_shape;
            s->~b2PolygonShape();
            allocator->Free(s, sizeof(b2PolygonShape));
        }
        break;

        default:
        b2Assert(false);
        break;
    }

    m_shape = nullptr;
}

void b2Fixture::CreateProxies(b2BroadPhase* broadPhase, const b2Transform& xf, bool active)
{
    b2FixtureProxy* proxy = m_proxies;
    proxy->xf = xf;
    if (!m_identity)
        proxy->xf = b2Mul(xf, m_xf);
    m_shape->ComputeAABB(&proxy->aabb, proxy->xf);
    proxy->proxyId = broadPhase->CreateProxy(proxy->aabb, proxy, active);
    proxy->fixture = this;
}

void b2Fixture::UpdateProxies(b2BroadPhase* broadPhase, bool active)
{
    b2FixtureProxy* proxy = m_proxies;
    broadPhase->UpdateProxy(proxy->proxyId, active);
}

void b2Fixture::DestroyProxies(b2BroadPhase* broadPhase)
{
    // Destroy proxies in the broad-phase.
    b2FixtureProxy* proxy = m_proxies;
    broadPhase->DestroyProxy(proxy->proxyId);
    proxy->proxyId = b2BroadPhase::e_nullProxy;
}

void b2Fixture::Update(b2BroadPhase* broadPhase, const b2Transform& xf)
{
    b2FixtureProxy* proxy = m_proxies;
    proxy->xf = xf;
    if (!m_identity)
        proxy->xf = b2Mul(xf, m_xf);
    m_shape->ComputeAABB(&proxy->aabb, proxy->xf);
    broadPhase->UpdateProxy(proxy->proxyId, proxy->aabb);
}
