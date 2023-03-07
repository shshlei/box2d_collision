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

#ifndef B2_FIXTURE_H
#define B2_FIXTURE_H

#include "b2_body.h"
#include "b2_collision.h"

class b2BlockAllocator;
class b2Body;
class b2BroadPhase;
class b2Fixture;

/// This holds contact filtering data.
struct B2_API b2Filter
{
    b2Filter()
    {
        categoryBits = 0x0001;
        maskBits = 0xFFFF;
        groupIndex = 0;
    }

    /// The collision category bits. Normally you would just set one bit.
    unsigned short categoryBits;

    /// The collision mask bits. This states the categories that this
    /// shape would accept for collision.
    unsigned short maskBits;

    /// Collision groups allow a certain group of objects to never collide (negative)
    /// or always collide (positive). Zero means no collision group. Non-zero group
    /// filtering always wins against the mask bits.
    short groupIndex;
};

/// A fixture definition is used to create a fixture. This class defines an
/// abstract fixture definition. You can reuse fixture definitions safely.
struct B2_API b2FixtureDef
{
    /// The constructor sets the default fixture definition values.
    b2FixtureDef()
    {
        shape = nullptr;
    }

    /// The shape, this must be set. The shape will be cloned, so you
    /// can create the shape on the stack.
    const b2Shape* shape;

    /// The local transform.
    b2Transform xf;

    /// Use this to store application specific fixture data.
    b2FixtureUserData userData;

    /// Contact filtering data.
    b2Filter filter;
};

/// This proxy is used internally to connect fixtures to the broad-phase.
struct B2_API b2FixtureProxy
{
    b2AABB aabb;        // un-expanded aabb
    b2Fixture* fixture;
    b2Transform xf;     // global transform
    int proxyId;
};

/// A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent.
/// Fixtures are created via b2Body::CreateFixture.
/// @warning you cannot reuse fixtures.
class B2_API b2Fixture
{
public:
    /// Get the type of the child shape. You can use this to down cast to the concrete shape.
    /// @return the shape type.
    b2Shape::Type GetType() const;

    /// Get the child shape. You can modify the child shape, however you should not change the
    /// number of vertices because this will crash some collision caching mechanisms.
    /// Manipulating the shape may lead to non-physical behavior.
    b2Shape* GetShape();
    const b2Shape* GetShape() const;
    
    /// Set the contact filtering data. This will not update contacts until the next time
    /// step when either parent body is active and awake.
    /// This automatically calls Refilter.
    void SetFilterData(const b2Filter& filter);

    /// Get the contact filtering data.
    const b2Filter& GetFilterData() const;

    /// Get the parent body of this fixture. This is nullptr if the fixture is not attached.
    /// @return the parent body.
    b2Body* GetBody();
    const b2Body* GetBody() const;

    /// Get the next fixture in the parent body's fixture list.
    /// @return the next shape.
    b2Fixture* GetNext();
    const b2Fixture* GetNext() const;

    void SetUserData(const b2FixtureUserData& userData);

    /// Get the user data that was assigned in the fixture definition. Use this to
    /// store your application specific data.
    b2FixtureUserData& GetUserData();
    const b2FixtureUserData& GetUserData() const;

    /// Test a point for containment in this fixture.
    /// @param p a point in world coordinates.
    bool TestPoint(const b2Vec2& p) const;

    /// Get the fixture's AABB. This AABB may be enlarge and/or stale.
    /// If you need a more accurate AABB, compute it using the shape and
    /// the body transform.
    const b2AABB& GetAABB() const;

    int GetProxyId() const;

    const b2Transform& GetLocalTransform() const;

    const b2Transform& GetGlobalTransform() const;

protected:

    friend class b2Body;
    friend class b2Contact;
    friend class b2BVHManager;

    b2Fixture();

    // We need separation create/destroy functions from the constructor/destructor because
    // the destructor cannot access the allocator (no destructor arguments allowed by C++).
    void Create(b2BlockAllocator* allocator, b2Body* body, const b2FixtureDef* def);
    void Destroy(b2BlockAllocator* allocator);

    // These support body activation/deactivation.
    void CreateProxies(b2BroadPhase* broadPhase, const b2Transform& xf, bool active = true);
    void UpdateProxies(b2BroadPhase* broadPhase, bool active);
    void DestroyProxies(b2BroadPhase* broadPhase);
    void Update(b2BroadPhase* broadPhase, const b2Transform& xf);

    b2Fixture* m_next;

    b2Shape* m_shape;
    b2Body* m_body;

    // Local transform
    b2Transform m_xf;
    bool m_identity;

    b2FixtureProxy* m_proxies;

    b2Filter m_filter;

    b2FixtureUserData m_userData;
};

B2_FORCE_INLINE b2Shape::Type b2Fixture::GetType() const
{
    return m_shape->GetType();
}

B2_FORCE_INLINE b2Shape* b2Fixture::GetShape()
{
    return m_shape;
}

B2_FORCE_INLINE const b2Shape* b2Fixture::GetShape() const
{
    return m_shape;
}

B2_FORCE_INLINE void b2Fixture::SetFilterData(const b2Filter& filter)
{
    m_filter = filter;
}

B2_FORCE_INLINE const b2Filter& b2Fixture::GetFilterData() const
{
    return m_filter;
}

B2_FORCE_INLINE void b2Fixture::SetUserData(const b2FixtureUserData& userData)
{
    m_userData = userData;
}

B2_FORCE_INLINE b2FixtureUserData& b2Fixture::GetUserData()
{
    return m_userData;
}

B2_FORCE_INLINE const b2FixtureUserData& b2Fixture::GetUserData() const
{
    return m_userData;
}

B2_FORCE_INLINE b2Body* b2Fixture::GetBody()
{
    return m_body;
}

B2_FORCE_INLINE const b2Body* b2Fixture::GetBody() const
{
    return m_body;
}

B2_FORCE_INLINE b2Fixture* b2Fixture::GetNext()
{
    return m_next;
}

B2_FORCE_INLINE const b2Fixture* b2Fixture::GetNext() const
{
    return m_next;
}

B2_FORCE_INLINE bool b2Fixture::TestPoint(const b2Vec2& p) const
{
    if (m_proxies->fixture)
        return m_shape->TestPoint(m_proxies->xf, p);
    if (m_identity)
        return m_shape->TestPoint(m_body->GetTransform(), p);
    else
        return m_shape->TestPoint(b2Mul(m_body->GetTransform(), m_xf), p);
}

B2_FORCE_INLINE const b2AABB& b2Fixture::GetAABB() const
{
    if (m_proxies->fixture)
        return m_proxies->aabb;
    b2Transform xf = m_body->GetTransform();
    if (!m_identity)
        xf = b2Mul(xf, m_xf);
    b2AABB aabb;
    m_shape->ComputeAABB(&aabb, xf);
    return aabb;
}

B2_FORCE_INLINE	int b2Fixture::GetProxyId() const
{
    return m_proxies->proxyId;
}

B2_FORCE_INLINE const b2Transform& b2Fixture::GetLocalTransform() const
{
    return m_xf;
}

B2_FORCE_INLINE const b2Transform& b2Fixture::GetGlobalTransform() const
{
    return m_proxies->xf;
}

#endif
