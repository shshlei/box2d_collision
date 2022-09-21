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
#include "box2d_collision/b2_contact.h"
#include "box2d_collision/b2_bvh_manager.h"

#include <new>
#include <algorithm>

b2ContactFilter b2_defaultFilter;

b2BVHManager::b2BVHManager()
{
    m_contactFilter = &b2_defaultFilter;

    m_bodyList = nullptr;
    m_bodyCount = 0;

    m_contactList = nullptr;
    m_contactCount = 0;

    b2BlockAllocator* allocator = &m_blockAllocator;
    void* memory = allocator->Allocate(sizeof(b2BroadPhase));
    m_broadPhase = new (memory) b2BroadPhase;
}

b2BVHManager::~b2BVHManager()
{
    while (m_bodyList)
    {
        DestroyBody(m_bodyList);
    }

    b2BlockAllocator* allocator = &m_blockAllocator;
    m_broadPhase->~b2BroadPhase();
    allocator->Free(m_broadPhase, sizeof(b2BroadPhase));
}

b2Body* b2BVHManager::AddBody(const std::string& name, const b2Shape* shape, bool active)
{
    return AddBody(name, std::vector<const b2Shape *>(1, shape), active);
}

b2Body* b2BVHManager::AddBody(const std::string& name, const std::vector<const b2Shape*>& shapes, bool active)
{
//    b2Assert(!name.empty());
//    b2Assert(!shapes.empty());

    b2Body* body = CreateBody(name, active);
    for (std::size_t i = 0; i < shapes.size(); i++)
    {
        const b2Shape *shape = shapes[i];
        b2Fixture* fixture = body->AddShape(shape, i);
        fixture->CreateProxies(m_broadPhase, body->GetTransform(), active);
    }

    return body;
}

void b2BVHManager::AddShapeToBody(b2Body *body, const b2Shape *shape, uint32 shape_index)
{
    b2Fixture* fixture = body->AddShape(shape, shape_index);
    fixture->CreateProxies(m_broadPhase, body->GetTransform(), body->IsActive());
}

b2Body* b2BVHManager::CreateBody(const std::string& name, bool active)
{
//    b2Assert(!name.empty());

    b2BodyDef bd(name);
    if (active)
        bd.type = b2_dynamicBody;
    b2Body* body = CreateBody(&bd);
    body->SetBlockAllocator(&m_blockAllocator);
    return body;
}

b2Body* b2BVHManager::CreateBody(const b2BodyDef* def)
{
    void* mem = m_blockAllocator.Allocate(sizeof(b2Body));
    b2Body* b = new (mem) b2Body(def);

    // Add to world doubly linked list.
    b->m_prev = nullptr;
    b->m_next = m_bodyList;
    if (m_bodyList)
        m_bodyList->m_prev = b;
    m_bodyList = b;
    ++m_bodyCount;

    return b;
}

bool b2BVHManager::HasBody(const std::string& name) const
{
    for (const b2Body* b = m_bodyList; b; b = b->m_next)
    {
        if (b->GetName() == name)
            return true;
    }

    return false;
}

bool b2BVHManager::RemoveBody(const std::string& name)
{
    b2Body* body = GetBody(name);
    if (body)
    {
        DestroyBody(body);
        return true;
    }
    return false;
}

void b2BVHManager::DestroyBody(b2Body* b)
{
//    b2Assert(m_bodyCount > 0);

    // Delete the attached fixtures. This destroys broad-phase proxies.
    b2Fixture* f = b->m_fixtureList;
    while (f)
    {
        b2Fixture* f0 = f;
        f = f->m_next;

        f0->DestroyProxies(m_broadPhase);
        f0->Destroy(&m_blockAllocator);
        f0->~b2Fixture();
        m_blockAllocator.Free(f0, sizeof(b2Fixture));

        b->m_fixtureList = f;
        b->m_fixtureCount -= 1;
    }
    b->m_fixtureList = nullptr;
    b->m_fixtureCount = 0;

    // Remove world body list.
    if (b->m_prev)
    {
        b->m_prev->m_next = b->m_next;
    }

    if (b->m_next)
    {
        b->m_next->m_prev = b->m_prev;
    }

    if (b == m_bodyList)
    {
        m_bodyList = b->m_next;
    }

    --m_bodyCount;
    b->~b2Body();
    m_blockAllocator.Free(b, sizeof(b2Body));
}

b2Body* b2BVHManager::GetBody(const std::string& name)
{
    for (b2Body* b = m_bodyList; b; b = b->m_next)
    {
        if (b->GetName() == name)
            return b;
    }

    return nullptr;
}

const b2Body* b2BVHManager::GetBody(const std::string& name) const
{
    for (const b2Body* b = m_bodyList; b; b = b->m_next)
    {
        if (b->GetName() == name)
            return b;
    }

    return nullptr;
}

bool b2BVHManager::EnableBody(const std::string& name)
{
    b2Body* body = GetBody(name);
    if (body)
    {
        body->SetEnabled(true);
        return true;
    }

    return false;
}

void b2BVHManager::EnableAll()
{
    for (b2Body* b = m_bodyList; b; b = b->m_next)
        b->SetEnabled(true);
}

bool b2BVHManager::DisableBody(const std::string& name)
{
    b2Body* body = GetBody(name);
    if (body)
    {
        body->SetEnabled(false);
        return true;
    }

    return false;
}

void b2BVHManager::DisableAll()
{
    for (b2Body* b = m_bodyList; b; b = b->m_next)
        b->SetEnabled(false);
}

bool b2BVHManager::IsBodyEnabled(const std::string& name) const
{
    const b2Body* body = GetBody(name);
    if (body)
        return body->IsEnabled();

    return false;
}

bool b2BVHManager::IsBodyActive(const std::string& name) const
{
    const b2Body* body = GetBody(name);
    if (body)
        return body->IsActive();

    return false;
}

void b2BVHManager::SetBodyActive(const std::string& name, bool active)
{
    b2Body* b = GetBody(name);
    if (b)
    {
        if (active)
        {
            if (!b->IsActive())
            {
                b2Fixture *f = b->GetFixtureList();
                while (f)
                {
                    f->UpdateProxies(m_broadPhase, true);
                    f = f->GetNext();
                }
                b->SetType(b2_dynamicBody);
            }
        }
        else if (b->IsActive()) 
        {
            b2Fixture *f = b->GetFixtureList();
            while (f)
            {
                f->UpdateProxies(m_broadPhase, false);
                f = f->GetNext();
            }           
            b->SetType(b2_staticBody);
        }
    }
}

void b2BVHManager::SetFilterData(const std::string& name, const b2Filter& filter)
{
    b2Body* body = GetBody(name);
    b2Fixture *f = body->GetFixtureList();
    while (f)
    {
        f->SetFilterData(filter);
        f = f->GetNext();
    }
}

void b2BVHManager::SetUserData(const std::string& name, const b2BodyUserData& userData)
{
    b2Body* body = GetBody(name);
    body->SetUserData(userData);
}

void b2BVHManager::SetBodyTransform(const std::string& name, const b2Transform& xf)
{
    b2Body* body = GetBody(name);
    if (body)
    {
        body->SetTransform(xf);
        for (b2Fixture* f = body->m_fixtureList; f; f = f->m_next)
        {
            f->Update(m_broadPhase, xf);
        }
    }
}

void b2BVHManager::SetActiveBodys(const std::vector<std::string>& names)
{
    for (b2Body* b = m_bodyList; b; b = b->m_next)
    {
        if (std::find(names.begin(), names.end(), b->GetName()) != names.end())
        {
            if (!b->IsActive())
            {
                b2Fixture *f = b->GetFixtureList();
                while (f)
                {
                    f->UpdateProxies(m_broadPhase, true);
                    f = f->GetNext();
                }
                b->SetType(b2_dynamicBody);
            }
        }
        else if (b->IsActive()) 
        {
            b2Fixture *f = b->GetFixtureList();
            while (f)
            {
                f->UpdateProxies(m_broadPhase, false);
                f = f->GetNext();
            }           

            b->SetType(b2_staticBody);
        }
    }
}

struct b2BVHManagerQueryWrapper
{
    bool QueryCallback(int32 proxyId)
    {
        b2FixtureProxy* proxy = (b2FixtureProxy*)broadPhase->GetUserData(proxyId);
        return callback->ReportFixture(proxy->fixture);
    }
    const b2BroadPhase* broadPhase;
    b2QueryCallback* callback;
};

void b2BVHManager::QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const
{
    b2BVHManagerQueryWrapper wrapper;
    wrapper.broadPhase = m_broadPhase;
    wrapper.callback = callback;
    m_broadPhase->Query(&wrapper, aabb);
}

void b2BVHManager::QueryPoint(b2QueryCallback* callback, const b2Vec2& aabb) const
{
    b2BVHManagerQueryWrapper wrapper;
    wrapper.broadPhase = m_broadPhase;
    wrapper.callback = callback;
    m_broadPhase->Query(&wrapper, aabb);
}

struct b2BVHManagerQueryWrapper2
{
    bool QueryCallback(int32 proxyIdA, int32 proxyIdB)
    {
        b2FixtureProxy* proxyA = (b2FixtureProxy*)broadPhaseA->GetUserData(proxyIdA);
        b2FixtureProxy* proxyB = (b2FixtureProxy*)broadPhaseB->GetUserData(proxyIdB);
        return callback->ReportFixture(proxyA->fixture, proxyB->fixture);
    }
    const b2BroadPhase* broadPhaseA;
    const b2BroadPhase* broadPhaseB;
    b2QueryCallback2* callback;
};

void b2BVHManager::QueryManager(b2QueryCallback2* callback, const b2BVHManager* manager) const
{
    b2BVHManagerQueryWrapper2 wrapper;
    wrapper.broadPhaseA = m_broadPhase;
    wrapper.broadPhaseB = manager->GetBroadPhase();
    wrapper.callback = callback;
    m_broadPhase->Query(&wrapper, manager->GetBroadPhase());
}

struct b2BVHManagerRayCastWrapper
{
    b2Scalar RayCastCallback(const b2RayCastInput& input, int32 proxyId)
    {
        void* userData = broadPhase->GetUserData(proxyId);
        b2FixtureProxy* proxy = (b2FixtureProxy*)userData;
        b2Fixture* fixture = proxy->fixture;
        b2RayCastOutput output;
        bool hit = fixture->RayCast(&output, input);

        if (hit)
        {
            b2Scalar fraction = output.fraction;
            b2Vec2 point = (b2Scalar(1.0) - fraction) * input.p1 + fraction * input.p2;
            return callback->ReportFixture(fixture, point, output.normal, fraction);
        }

        return input.maxFraction;
    }

    const b2BroadPhase* broadPhase;
    b2RayCastCallback* callback;
};

void b2BVHManager::RayCast(b2RayCastCallback* callback, const b2Vec2& point1, const b2Vec2& point2) const
{
    b2BVHManagerRayCastWrapper wrapper;
    wrapper.broadPhase = m_broadPhase;
    wrapper.callback = callback;
    b2RayCastInput input;
    input.maxFraction = b2Scalar(1.0);
    input.p1 = point1;
    input.p2 = point2;
    m_broadPhase->RayCast(&wrapper, input);
}

bool b2BVHManager::ContactTest(b2Manifold* worldManifold, b2InscribedSpheres* inscribedSpheres)
{
    bool collision = false;
    m_broadPhase->UpdatePairs(this);
    b2Contact* c = m_contactList;
    while (c)
    {
        collision = CalculateContactResult(c, worldManifold, inscribedSpheres);
        if (collision)
            break;
        c = c->GetNext();
    }

    while (m_contactList)
    {
        if (collision)
        {
            b2Fixture* fixtureA = m_contactList->GetFixtureA();
            b2Fixture* fixtureB = m_contactList->GetFixtureB();
            b2Body* bodyA = fixtureA->GetBody();
            b2Body* bodyB = fixtureB->GetBody();
            if (bodyA->IsActive())
                m_broadPhase->TouchProxy(fixtureA->GetProxyId());
            if (bodyB->IsActive())
                m_broadPhase->TouchProxy(fixtureB->GetProxyId());
        }
        DestroyContact(m_contactList);
    }

    return collision;
}

bool b2BVHManager::ContactTest(b2ContactResult* contacts, b2InscribedSpheres* inscribedSpheres)
{
    bool collision = false;
    m_broadPhase->UpdatePairs(this);
    b2Contact* c = m_contactList;
    while (c)
    {
        collision = CalculateContactResult(c, contacts, inscribedSpheres);
        if (collision)
            break;
        c = c->GetNext();
    }

    while (m_contactList)
    {
        if (collision)
        {
            b2Fixture* fixtureA = m_contactList->GetFixtureA();
            b2Fixture* fixtureB = m_contactList->GetFixtureB();
            b2Body* bodyA = fixtureA->GetBody();
            b2Body* bodyB = fixtureB->GetBody();
            if (bodyA->IsActive())
                m_broadPhase->TouchProxy(fixtureA->GetProxyId());
            if (bodyB->IsActive())
                m_broadPhase->TouchProxy(fixtureB->GetProxyId());
        }
        DestroyContact(m_contactList);
    }

    return collision;
}

bool b2BVHManager::ContactTest(b2BVHManager *manager, b2Manifold* worldManifold, b2InscribedSpheres* inscribedSpheres)
{
    if (ContactTest(worldManifold, inscribedSpheres))
        return true;
    if (manager->ContactTest(worldManifold, inscribedSpheres))
        return true;

    struct b2BVHManagerQuery2
    {
        bool QueryCallback(int32 proxyIdA, int32 proxyIdB)
        {
            manager->AddPair(broadPhaseA->GetUserData(proxyIdA), broadPhaseB->GetUserData(proxyIdB));
            b2Contact* c = manager->GetContactList();
            if (c)
            {
                collision = manager->CalculateContactResult(c, worldManifold, inscribedSpheres);
                manager->DestroyContact(c);
            }
            return !collision;
        }
        bool collision{false};
        b2BVHManager *manager;
        const b2BroadPhase* broadPhaseA;
        const b2BroadPhase* broadPhaseB;
        b2Manifold* worldManifold;
        b2InscribedSpheres* inscribedSpheres;
    };

    b2BVHManagerQuery2 query;
    query.manager = this;
    query.broadPhaseA = GetBroadPhase();
    query.broadPhaseB = manager->GetBroadPhase();
    query.worldManifold = worldManifold;
    query.inscribedSpheres = inscribedSpheres;
    m_broadPhase->Query(&query, manager->GetBroadPhase());
    return query.collision;
}

bool b2BVHManager::ContactTest(b2BVHManager *manager, b2ContactResult* contacts, b2InscribedSpheres* inscribedSpheres)
{
    if (ContactTest(contacts, inscribedSpheres))
        return true;
    if (manager->ContactTest(contacts, inscribedSpheres))
        return true;

    struct b2BVHManagerQuery2
    {
        bool QueryCallback(int32 proxyIdA, int32 proxyIdB)
        {
            manager->AddPair(broadPhaseA->GetUserData(proxyIdA), broadPhaseB->GetUserData(proxyIdB));
            b2Contact* c = manager->GetContactList();
            if (c)
            {
                collision = manager->CalculateContactResult(c, contacts, inscribedSpheres);
                manager->DestroyContact(c);
            }
            return !collision;
        }
        bool collision{false};
        b2BVHManager *manager;
        const b2BroadPhase* broadPhaseA;
        const b2BroadPhase* broadPhaseB;
        b2ContactResult* contacts;
        b2InscribedSpheres* inscribedSpheres;
    };

    b2BVHManagerQuery2 query;
    query.manager = this;
    query.broadPhaseA = GetBroadPhase();
    query.broadPhaseB = manager->GetBroadPhase();
    query.contacts = contacts;
    query.inscribedSpheres = inscribedSpheres;
    m_broadPhase->Query(&query, manager->GetBroadPhase());
    return query.collision;
}

bool b2BVHManager::CalculateContactResult(b2Contact* c, b2Manifold* worldManifold, b2InscribedSpheres* inscribedSpheres) const
{
    bool collision = false;
    b2Fixture* fixtureA = c->GetFixtureA();
    b2Fixture* fixtureB = c->GetFixtureB();
    b2Body* bodyA = fixtureA->GetBody();
    b2Body* bodyB = fixtureB->GetBody();
    const b2Transform& xfA = bodyA->GetTransform();
    const b2Transform& xfB = bodyB->GetTransform();
    if (worldManifold)
    {
        if (c->Evaluate(worldManifold, xfA, xfB))
        {
            collision = true;
            if (inscribedSpheres)
                inscribedSpheres->Initialize(worldManifold, fixtureA->GetShape(), xfA, fixtureB->GetShape(), xfB);
        }
    }
    else 
    {
        if (c->Evaluate(nullptr, xfA, xfB))
            collision = true;
    }
    return collision;
}

bool b2BVHManager::CalculateContactResult(b2Contact* c, b2ContactResult* contacts, b2InscribedSpheres* inscribedSpheres) const
{
    bool collision = false;
    b2Fixture* fixtureA = c->GetFixtureA();
    b2Fixture* fixtureB = c->GetFixtureB();
    b2Body* bodyA = fixtureA->GetBody();
    b2Body* bodyB = fixtureB->GetBody();
    const b2Transform& xfA = bodyA->GetTransform();
    const b2Transform& xfB = bodyB->GetTransform();
    if (contacts)
    {
        b2Manifold worldManifold;
        if (c->Evaluate(&worldManifold, xfA, xfB))
        {
            collision = true;
            contacts->names[0] = bodyA->GetName();
            contacts->names[1] = bodyB->GetName();
            contacts->shape_id[0] = fixtureA->GetUserData().pointer;
            contacts->shape_id[1] = fixtureB->GetUserData().pointer;
            contacts->transforms[0] = xfA;
            contacts->transforms[1] = xfB;
            contacts->normal = worldManifold.normal;
            contacts->separation = worldManifold.separation;
            contacts->points[0] = worldManifold.point - worldManifold.separation * worldManifold.normal;
            contacts->points[1] = worldManifold.point;
            contacts->local_points[0] = b2MulT(xfA.q, contacts->points[0] - xfA.p);
            contacts->local_points[1] = b2MulT(xfB.q, contacts->points[1] - xfB.p);
            if (inscribedSpheres)
                inscribedSpheres->Initialize(contacts, fixtureA->GetShape(), fixtureB->GetShape());
        }
    }
    else 
    {
        if (c->Evaluate(nullptr, xfA, xfB))
            collision = true;
    }
    return collision;
}

void b2BVHManager::DestroyContact(b2Contact* c)
{
    // Remove from the world.
    if (c->m_prev)
    {
        c->m_prev->m_next = c->m_next;
    }

    if (c->m_next)
    {
        c->m_next->m_prev = c->m_prev;
    }

    if (c == m_contactList)
    {
        m_contactList = c->m_next;
    }

    // Call the factory.
    b2Contact::Destroy(c, &m_blockAllocator);
    --m_contactCount;
}

void b2BVHManager::AddPair(void* proxyUserDataA, void* proxyUserDataB)
{
    b2FixtureProxy* proxyA = (b2FixtureProxy*)proxyUserDataA;
    b2FixtureProxy* proxyB = (b2FixtureProxy*)proxyUserDataB;

    b2Fixture* fixtureA = proxyA->fixture;
    b2Fixture* fixtureB = proxyB->fixture;

    b2Body* bodyA = fixtureA->GetBody();
    b2Body* bodyB = fixtureB->GetBody();

    // Are the fixtures on the same body?
    if (bodyA == bodyB)
        return;

    if (!bodyA->IsEnabled() || !bodyB->IsEnabled())
        return;

    // Does a joint override collision? Is at least one body dynamic?
    if (!bodyB->ShouldCollide(bodyA))
        return;

    // Check user filtering.
    if (m_contactFilter && !m_contactFilter->ShouldCollide(fixtureA, fixtureB))
        return;

    if (!b2TestOverlap(proxyA->aabb, proxyB->aabb))
        return;

    // Call the factory.
    b2Contact* c = b2Contact::Create(fixtureA, fixtureB, &m_blockAllocator);
    if (!c)
        return;

    // Insert into the world.
    c->m_prev = nullptr;
    c->m_next = m_contactList;
    if (m_contactList != nullptr)
        m_contactList->m_prev = c;
    m_contactList = c;

    ++m_contactCount;
}

b2Scalar b2BVHManager::DistanceTest(b2Manifold* worldManifold)
{
    b2Scalar distance = b2_maxFloat;
    m_broadPhase->UpdateDistancePairs(this);
    while (m_contactList)
    {
        b2Manifold temp;
        CalculateDistanceResult(m_contactList, &temp);
        if (temp.separation < distance)
        {
            distance = temp.separation;
            if (worldManifold)
                *worldManifold = temp;
        }
        DestroyContact(m_contactList);
    }

    return distance;
}

b2Scalar b2BVHManager::DistanceTest(b2ContactResult* contacts)
{
    b2Scalar distance = b2_maxFloat;
    m_broadPhase->UpdateDistancePairs(this);
    while (m_contactList)
    {
        b2ContactResult temp;
        CalculateDistanceResult(m_contactList, &temp);
        if (temp.separation < distance)
        {
            distance = temp.separation;
            if (contacts)
                *contacts = temp;
        }
        DestroyContact(m_contactList);
    }

    return distance;
}

void b2BVHManager::CalculateDistanceResult(b2Contact* c, b2Manifold* worldManifold) const
{
    b2Fixture* fixtureA = c->GetFixtureA();
    b2Fixture* fixtureB = c->GetFixtureB();
    b2Body* bodyA = fixtureA->GetBody();
    b2Body* bodyB = fixtureB->GetBody();
    const b2Transform& xfA = bodyA->GetTransform();
    const b2Transform& xfB = bodyB->GetTransform();
    c->Evaluate(worldManifold, xfA, xfB);
}

void b2BVHManager::CalculateDistanceResult(b2Contact* c, b2ContactResult* contacts) const
{
    b2Fixture* fixtureA = c->GetFixtureA();
    b2Fixture* fixtureB = c->GetFixtureB();
    b2Body* bodyA = fixtureA->GetBody();
    b2Body* bodyB = fixtureB->GetBody();
    const b2Transform& xfA = bodyA->GetTransform();
    const b2Transform& xfB = bodyB->GetTransform();
    b2Manifold worldManifold;
    c->Evaluate(&worldManifold, xfA, xfB);

    contacts->names[0] = bodyA->GetName();
    contacts->names[1] = bodyB->GetName();
    contacts->shape_id[0] = fixtureA->GetUserData().pointer;
    contacts->shape_id[1] = fixtureB->GetUserData().pointer;
    contacts->transforms[0] = xfA;
    contacts->transforms[1] = xfB;
    contacts->normal = worldManifold.normal;
    contacts->separation = worldManifold.separation;
    contacts->points[0] = worldManifold.point - worldManifold.separation * worldManifold.normal;
    contacts->points[1] = worldManifold.point;
    contacts->local_points[0] = b2MulT(xfA.q, contacts->points[0] - xfA.p);
    contacts->local_points[1] = b2MulT(xfB.q, contacts->points[1] - xfB.p);
}

void b2BVHManager::AddPairDistance(void* proxyUserDataA, void* proxyUserDataB)
{
    b2FixtureProxy* proxyA = (b2FixtureProxy*)proxyUserDataA;
    b2FixtureProxy* proxyB = (b2FixtureProxy*)proxyUserDataB;

    b2Fixture* fixtureA = proxyA->fixture;
    b2Fixture* fixtureB = proxyB->fixture;

    b2Body* bodyA = fixtureA->GetBody();
    b2Body* bodyB = fixtureB->GetBody();

    // Are the fixtures on the same body?
    if (bodyA == bodyB)
        return;

    if (!bodyA->IsEnabled() || !bodyB->IsEnabled())
        return;

    // Does a joint override collision? Is at least one body dynamic?
    if (!bodyB->ShouldCollide(bodyA))
        return;

    // Check user filtering.
    if (m_contactFilter && !m_contactFilter->ShouldCollide(fixtureA, fixtureB))
        return;

    // Call the factory.
    b2Contact* c = b2Contact::Create(fixtureA, fixtureB, &m_blockAllocator);
    if (!c)
        return;

    // Insert into the world.
    c->m_prev = nullptr;
    c->m_next = m_contactList;
    if (m_contactList != nullptr)
        m_contactList->m_prev = c;
    m_contactList = c;

    ++m_contactCount;
}
