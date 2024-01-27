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

#include "box2d_collision/b2_bvh_manager.h"

#include <algorithm>
#include <new>

b2ContactFilter b2_defaultFilter;

b2BVHManager::b2BVHManager()
{
  m_contactFilter = &b2_defaultFilter;

  m_bodyList = nullptr;
  m_bodyCount = 0;

  b2BlockAllocator * allocator = &m_blockAllocator;
  void * memory = allocator->Allocate(sizeof(b2BroadPhase));
  m_broadPhase = new (memory) b2BroadPhase;
}

b2BVHManager::~b2BVHManager()
{
  while (m_bodyList) {
    DestroyBody(m_bodyList);
  }

  b2BlockAllocator * allocator = &m_blockAllocator;
  m_broadPhase->~b2BroadPhase();
  allocator->Free(m_broadPhase, sizeof(b2BroadPhase));
}

void b2BVHManager::SetContactFilter(b2ContactFilter * filter)
{
  m_contactFilter = filter;
}

const b2ContactFilter * b2BVHManager::GetContactFilter() const
{
  return m_contactFilter;
}

b2ContactFilter * b2BVHManager::GetContactFilter()
{
  return m_contactFilter;
}

b2Body * b2BVHManager::GetBodyList()
{
  return m_bodyList;
}

const b2Body * b2BVHManager::GetBodyList() const
{
  return m_bodyList;
}

int b2BVHManager::GetBodyCount() const
{
  return m_bodyCount;
}

const b2BroadPhase * b2BVHManager::GetBroadPhase() const
{
  return m_broadPhase;
}

int b2BVHManager::GetProxyCount() const
{
  return m_broadPhase->GetProxyCount();
}

int b2BVHManager::GetTreeHeight() const
{
  return m_broadPhase->GetTreeHeight();
}

int b2BVHManager::GetTreeBalance() const
{
  return m_broadPhase->GetTreeBalance();
}

b2Scalar b2BVHManager::GetTreeQuality() const
{
  return m_broadPhase->GetTreeQuality();
}

void b2BVHManager::ShiftOrigin(const b2Vec2 & newOrigin)
{
  for (b2Body * b = m_bodyList; b; b = b->m_next) {
    b->m_xf.translation() -= newOrigin;
  }

  m_broadPhase->ShiftOrigin(newOrigin);
}

void b2BVHManager::setContactDistanceThreshold(b2Scalar contact_distance)
{
  m_broadPhase->setContactDistanceThreshold(contact_distance);
}

b2Scalar b2BVHManager::getContactDistanceThreshold() const
{
  return m_broadPhase->getContactDistanceThreshold();
}

b2Body * b2BVHManager::AddBody(const std::string & name, const b2Shape * shape, const b2Transform & xf, bool active)
{
  return AddBody(name, std::vector<const b2Shape *>(1, shape), std::vector<b2Transform>(1, xf), active);
}

b2Body * b2BVHManager::AddBody(const std::string & name, const std::vector<const b2Shape *> & shapes, const std::vector<b2Transform> & xfs, bool active)
{
  //    b2Assert(!name.empty());
  //    b2Assert(!shapes.empty());

  b2Body * body = CreateBody(name, active);
  for (std::size_t i = 0; i < shapes.size(); i++) {
    const b2Shape * shape = shapes[i];
    b2Fixture * fixture = body->AddShape(shape, xfs[i], i);
    fixture->CreateProxies(m_broadPhase, body->GetTransform(), active);
  }

  return body;
}

void b2BVHManager::AddShapeToBody(b2Body * body, const b2Shape * shape, const b2Transform & xf, unsigned int shape_index)
{
  b2Fixture * fixture = body->AddShape(shape, xf, shape_index);
  fixture->CreateProxies(m_broadPhase, body->GetTransform(), body->IsActive());
}

b2Body * b2BVHManager::CreateBody(const std::string & name, bool active)
{
  //    b2Assert(!name.empty());

  b2BodyDef bd(name);
  if (active)
    bd.type = b2_dynamicBody;
  b2Body * body = CreateBody(&bd);
  body->SetBlockAllocator(&m_blockAllocator);
  return body;
}

b2Body * b2BVHManager::CreateBody(const b2BodyDef * def)
{
  void * mem = m_blockAllocator.Allocate(sizeof(b2Body));
  b2Body * b = new (mem) b2Body(def);

  // Add to world doubly linked list.
  b->m_prev = nullptr;
  b->m_next = m_bodyList;
  if (m_bodyList)
    m_bodyList->m_prev = b;
  m_bodyList = b;
  ++m_bodyCount;

  return b;
}

void b2BVHManager::DestroyBody(b2Body * b)
{
  //    b2Assert(m_bodyCount > 0);

  // Delete the attached fixtures. This destroys broad-phase proxies.
  b2Fixture * f = b->m_fixtureList;
  while (f) {
    b2Fixture * f0 = f;
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
  if (b->m_prev) {
    b->m_prev->m_next = b->m_next;
  }

  if (b->m_next) {
    b->m_next->m_prev = b->m_prev;
  }

  if (b == m_bodyList) {
    m_bodyList = b->m_next;
  }

  --m_bodyCount;
  b->~b2Body();
  m_blockAllocator.Free(b, sizeof(b2Body));
}

bool b2BVHManager::HasBody(const std::string & name) const
{
  for (const b2Body * b = m_bodyList; b; b = b->m_next) {
    if (b->GetName() == name)
      return true;
  }

  return false;
}

bool b2BVHManager::RemoveBody(const std::string & name)
{
  b2Body * body = GetBody(name);
  if (body) {
    DestroyBody(body);
    return true;
  }
  return false;
}

b2Body * b2BVHManager::GetBody(const std::string & name)
{
  for (b2Body * b = m_bodyList; b; b = b->m_next) {
    if (b->GetName() == name)
      return b;
  }

  return nullptr;
}

const b2Body * b2BVHManager::GetBody(const std::string & name) const
{
  for (const b2Body * b = m_bodyList; b; b = b->m_next) {
    if (b->GetName() == name)
      return b;
  }

  return nullptr;
}

bool b2BVHManager::EnableBody(const std::string & name)
{
  b2Body * body = GetBody(name);
  if (body) {
    body->SetEnabled(true);
    return true;
  }

  return false;
}

void b2BVHManager::EnableAll()
{
  for (b2Body * b = m_bodyList; b; b = b->m_next)
    b->SetEnabled(true);
}

bool b2BVHManager::DisableBody(const std::string & name)
{
  b2Body * body = GetBody(name);
  if (body) {
    body->SetEnabled(false);
    return true;
  }

  return false;
}

void b2BVHManager::DisableAll()
{
  for (b2Body * b = m_bodyList; b; b = b->m_next)
    b->SetEnabled(false);
}

bool b2BVHManager::IsBodyEnabled(const std::string & name) const
{
  const b2Body * body = GetBody(name);
  if (body)
    return body->IsEnabled();

  return false;
}

bool b2BVHManager::IsBodyActive(const std::string & name) const
{
  const b2Body * body = GetBody(name);
  if (body)
    return body->IsActive();

  return false;
}

void b2BVHManager::SetBodyActive(const std::string & name, bool active)
{
  b2Body * b = GetBody(name);
  if (b) {
    if (active) {
      if (!b->IsActive()) {
        b2Fixture * f = b->GetFixtureList();
        while (f) {
          f->UpdateProxies(m_broadPhase, true);
          f = f->GetNext();
        }
        b->SetType(b2_dynamicBody);
      }
    }
    else if (b->IsActive()) {
      b2Fixture * f = b->GetFixtureList();
      while (f) {
        f->UpdateProxies(m_broadPhase, false);
        f = f->GetNext();
      }
      b->SetType(b2_staticBody);
    }
  }
}

void b2BVHManager::SetActiveBodys(const std::vector<std::string> & names)
{
  for (b2Body * b = m_bodyList; b; b = b->m_next) {
    if (std::find(names.begin(), names.end(), b->GetName()) != names.end()) {
      if (!b->IsActive()) {
        b2Fixture * f = b->GetFixtureList();
        while (f) {
          f->UpdateProxies(m_broadPhase, true);
          f = f->GetNext();
        }
        b->SetType(b2_dynamicBody);
      }
    }
    else if (b->IsActive()) {
      b2Fixture * f = b->GetFixtureList();
      while (f) {
        f->UpdateProxies(m_broadPhase, false);
        f = f->GetNext();
      }
      b->SetType(b2_staticBody);
    }
  }
}

void b2BVHManager::SetFilterData(const std::string & name, const b2Filter & filter)
{
  b2Body * body = GetBody(name);
  b2Fixture * f = body->GetFixtureList();
  while (f) {
    f->SetFilterData(filter);
    f = f->GetNext();
  }
}

void b2BVHManager::SetUserData(const std::string & name, const b2BodyUserData & userData)
{
  b2Body * body = GetBody(name);
  body->SetUserData(userData);
}

void b2BVHManager::SetBodyTransform(const std::string & name, const b2Transform & xf)
{
  b2Body * body = GetBody(name);
  if (body) {
    body->SetTransform(xf);
    for (b2Fixture * f = body->m_fixtureList; f; f = f->m_next) {
      f->Update(m_broadPhase, xf);
    }
  }
}

struct b2BVHManagerCallback
{
  bool CollideCallback(int proxyId)
  {
    b2FixtureProxy * proxy = (b2FixtureProxy *)broadPhase->GetUserData(proxyId);
    return callback->ReportCollision(proxy->fixture);
  }
  bool DistanceCallback(int proxyId, b2Scalar & dist)
  {
    b2FixtureProxy * proxy = (b2FixtureProxy *)broadPhase->GetUserData(proxyId);
    return callback->ReportDistance(proxy->fixture, dist);
  }
  bool DistanceCallback(int proxyIdA, int proxyIdB, b2Scalar & dist)
  {
    b2FixtureProxy * proxyA = (b2FixtureProxy *)broadPhase->GetUserData(proxyIdA);
    b2FixtureProxy * proxyB = (b2FixtureProxy *)broadPhase->GetUserData(proxyIdB);
    return callback->ReportDistance(proxyA->fixture, proxyB->fixture, dist);
  }
  const b2BroadPhase * broadPhase{nullptr};
  b2NaiveCallback * callback{nullptr};
};

struct b2BVHManagerCallback2
{
  bool CollideCallback(int proxyIdA, int proxyIdB)
  {
    b2FixtureProxy * proxyA = (b2FixtureProxy *)broadPhaseA->GetUserData(proxyIdA);
    b2FixtureProxy * proxyB = (b2FixtureProxy *)broadPhaseB->GetUserData(proxyIdB);
    return callback->ReportCollision(proxyA->fixture, proxyB->fixture);
  }
  bool DistanceCallback(int proxyIdA, int proxyIdB, b2Scalar & dist)
  {
    b2FixtureProxy * proxyA = (b2FixtureProxy *)broadPhaseA->GetUserData(proxyIdA);
    b2FixtureProxy * proxyB = (b2FixtureProxy *)broadPhaseB->GetUserData(proxyIdB);
    return callback->ReportDistance(proxyA->fixture, proxyB->fixture, dist);
  }
  const b2BroadPhase * broadPhaseA{nullptr};
  const b2BroadPhase * broadPhaseB{nullptr};
  b2NaiveCallback * callback{nullptr};
};

bool b2BVHManager::Collide(b2NaiveCallback * callback, const b2AABB & aabb) const
{
  b2BVHManagerCallback wrapper;
  wrapper.broadPhase = m_broadPhase;
  wrapper.callback = callback;
  return m_broadPhase->Collide(&wrapper, aabb);
}

bool b2BVHManager::Collide(b2NaiveCallback * callback, const b2OBB & aabb) const
{
  b2BVHManagerCallback wrapper;
  wrapper.broadPhase = m_broadPhase;
  wrapper.callback = callback;
  return m_broadPhase->Collide(&wrapper, aabb);
}

bool b2BVHManager::Collide(b2NaiveCallback * callback, const b2Vec2 & point) const
{
  b2BVHManagerCallback wrapper;
  wrapper.broadPhase = m_broadPhase;
  wrapper.callback = callback;
  return m_broadPhase->Collide(&wrapper, point);
}

bool b2BVHManager::Collide(b2NaiveCallback * callback, const b2BVHManager * manager) const
{
  b2BVHManagerCallback2 wrapper;
  wrapper.broadPhaseA = m_broadPhase;
  wrapper.broadPhaseB = manager->GetBroadPhase();
  wrapper.callback = callback;
  return m_broadPhase->Collide(&wrapper, manager->GetBroadPhase());
}

bool b2BVHManager::Distance(b2NaiveCallback * callback, const b2AABB & aabb, b2Scalar & dist) const
{
  b2BVHManagerCallback wrapper;
  wrapper.broadPhase = m_broadPhase;
  wrapper.callback = callback;
  return m_broadPhase->Distance(&wrapper, aabb, dist);
}

bool b2BVHManager::Distance(b2NaiveCallback * callback, const b2Vec2 & point, b2Scalar & dist) const
{
  b2BVHManagerCallback wrapper;
  wrapper.broadPhase = m_broadPhase;
  wrapper.callback = callback;
  return m_broadPhase->Distance(&wrapper, point, dist);
}

bool b2BVHManager::Distance(b2NaiveCallback * callback, const b2BVHManager * manager, b2Scalar & dist) const
{
  b2BVHManagerCallback2 wrapper;
  wrapper.broadPhaseA = m_broadPhase;
  wrapper.broadPhaseB = manager->GetBroadPhase();
  wrapper.callback = callback;
  return m_broadPhase->Distance(&wrapper, manager->GetBroadPhase(), dist);
}

bool b2BVHManager::SelfDistance(b2NaiveCallback * callback, b2Scalar & dist) const
{
  b2BVHManagerCallback wrapper;
  wrapper.broadPhase = m_broadPhase;
  wrapper.callback = callback;
  return m_broadPhase->SelfDistance(&wrapper, dist);
}

bool CollisionResult(const b2FixtureProxy * proxyA, const b2FixtureProxy * proxyB,
  b2BVHManager * manager,
  b2Manifold * manifold,
  b2ContactResult * contacts,
  b2InscribedSpheres * inscribedSpheres)
{
  b2Fixture * fixtureA = proxyA->fixture;
  b2Fixture * fixtureB = proxyB->fixture;

  b2Body * bodyA = fixtureA->GetBody();
  b2Body * bodyB = fixtureB->GetBody();

  // Are the fixtures on the same body?
  if (bodyA == bodyB)
    return false;

  if (!bodyA->IsEnabled() || !bodyB->IsEnabled())
    return false;

  // Does a joint override collision? Is at least one body dynamic?
  if (!bodyB->ShouldCollide(bodyA))
    return false;

  // Check user filtering.
  if (manager->GetContactFilter() && !manager->GetContactFilter()->ShouldCollide(fixtureA, fixtureB))
    return false;

  if (!b2TestOverlap(fixtureA->GetAABB(), fixtureB->GetAABB()))
    return false;

  const b2Transform & xfA = fixtureA->GetGlobalTransform();
  const b2Transform & xfB = fixtureB->GetGlobalTransform();
  bool collision = b2CollideShapes(manifold, fixtureA->GetShape(), xfA, fixtureB->GetShape(), xfB);
  if (collision && manifold) {
    if (contacts) {
      contacts->names[0] = bodyA->GetName();
      contacts->names[1] = bodyB->GetName();
      contacts->shape_id[0] = fixtureA->GetUserData().pointer;
      contacts->shape_id[1] = fixtureB->GetUserData().pointer;
      contacts->transforms[0] = xfA;
      contacts->transforms[1] = xfB;
      contacts->normal = manifold->normal;
      contacts->separation = manifold->separation;
      contacts->points[0] = manifold->point - manifold->separation * manifold->normal;
      contacts->points[1] = manifold->point;
      contacts->local_points[0] = b2MulT(xfA, contacts->points[0]);
      contacts->local_points[1] = b2MulT(xfB, contacts->points[1]);
    }
    if (inscribedSpheres) {
      if (contacts)
        inscribedSpheres->Initialize(contacts, fixtureA->GetShape(), fixtureB->GetShape());
      else
        inscribedSpheres->Initialize(manifold, fixtureA->GetShape(), xfA, fixtureB->GetShape(), xfB);
    }
  }
  return collision;
}

/// Self collision test.
struct b2BVHManagerActiveCollideWrapper
{
  bool CollideCallback(void * proxyUserDataA, void * proxyUserDataB)
  {
    b2FixtureProxy * proxyA = (b2FixtureProxy *)proxyUserDataA;
    b2FixtureProxy * proxyB = (b2FixtureProxy *)proxyUserDataB;
    return CollisionResult(proxyA, proxyB, manager, manifold, contacts, inscribedSpheres);
  }
  b2BVHManager * manager;
  b2Manifold * manifold{nullptr};
  b2InscribedSpheres * inscribedSpheres{nullptr};
  b2ContactResult * contacts{nullptr};
};

struct b2BVHManagerActiveCollideWrapper2
{
  bool CollideCallback(int proxyIdA, int proxyIdB)
  {
    b2FixtureProxy * proxyA = (b2FixtureProxy *)broadPhaseA->GetUserData(proxyIdA);
    b2FixtureProxy * proxyB = (b2FixtureProxy *)broadPhaseB->GetUserData(proxyIdB);
    return CollisionResult(proxyA, proxyB, manager, manifold, contacts, inscribedSpheres);
  }
  b2BVHManager * manager{nullptr};
  const b2BroadPhase * broadPhaseA{nullptr};
  const b2BroadPhase * broadPhaseB{nullptr};
  b2Manifold * manifold{nullptr};
  b2InscribedSpheres * inscribedSpheres{nullptr};
  b2ContactResult * contacts{nullptr};
};

bool b2BVHManager::Collide(b2Manifold * worldManifold, b2InscribedSpheres * inscribedSpheres)
{
  b2BVHManagerActiveCollideWrapper wrapper;
  wrapper.manager = this;
  wrapper.manifold = worldManifold;
  wrapper.inscribedSpheres = inscribedSpheres;
  return m_broadPhase->Collide(&wrapper);
}

bool b2BVHManager::Collide(b2ContactResult * contacts, b2InscribedSpheres * inscribedSpheres)
{
  b2BVHManagerActiveCollideWrapper wrapper;
  wrapper.manager = this;
  wrapper.contacts = contacts;
  wrapper.inscribedSpheres = inscribedSpheres;
  if (contacts) {
    b2Manifold manifold;
    wrapper.manifold = &manifold;
  }
  return m_broadPhase->Collide(&wrapper);
}

bool b2BVHManager::Collide(b2BVHManager * manager, b2Manifold * worldManifold, b2InscribedSpheres * inscribedSpheres)
{
  if (Collide(worldManifold, inscribedSpheres))  // self one
    return true;
  if (manager->Collide(worldManifold, inscribedSpheres))  // self two
    return true;
  b2BVHManagerActiveCollideWrapper2 query;
  query.manager = this;
  query.broadPhaseA = GetBroadPhase();
  query.broadPhaseB = manager->GetBroadPhase();
  query.manifold = worldManifold;
  query.inscribedSpheres = inscribedSpheres;
  return m_broadPhase->Collide(&query, manager->GetBroadPhase());
}

bool b2BVHManager::Collide(b2BVHManager * manager, b2ContactResult * contacts, b2InscribedSpheres * inscribedSpheres)
{
  if (Collide(contacts, inscribedSpheres))  // self one
    return true;
  if (manager->Collide(contacts, inscribedSpheres))  // self two
    return true;
  b2BVHManagerActiveCollideWrapper2 query;
  query.manager = this;
  query.broadPhaseA = GetBroadPhase();
  query.broadPhaseB = manager->GetBroadPhase();
  query.contacts = contacts;
  query.inscribedSpheres = inscribedSpheres;
  if (contacts) {
    b2Manifold manifold;
    query.manifold = &manifold;
  }
  return m_broadPhase->Collide(&query, manager->GetBroadPhase());
}

bool DistanceResult(const b2FixtureProxy * proxyA, const b2FixtureProxy * proxyB,
  b2BVHManager * manager,
  b2Manifold * manifold,
  b2ContactResult * contacts)
{
  b2Fixture * fixtureA = proxyA->fixture;
  b2Fixture * fixtureB = proxyB->fixture;

  b2Body * bodyA = fixtureA->GetBody();
  b2Body * bodyB = fixtureB->GetBody();

  // Are the fixtures on the same body?
  if (bodyA == bodyB)
    return true;

  if (!bodyA->IsEnabled() || !bodyB->IsEnabled())
    return true;

  // Does a joint override collision? Is at least one body dynamic?
  if (!bodyB->ShouldCollide(bodyA))
    return true;

  // Check user filtering.
  if (manager->GetContactFilter() && !manager->GetContactFilter()->ShouldCollide(fixtureA, fixtureB))
    return true;

  const b2Transform & xfA = fixtureA->GetGlobalTransform();
  const b2Transform & xfB = fixtureB->GetGlobalTransform();
  bool collision = b2CollideShapes(manifold, fixtureA->GetShape(), xfA, fixtureB->GetShape(), xfB, false);
  if (collision && manifold) {
    if (contacts) {
      contacts->names[0] = bodyA->GetName();
      contacts->names[1] = bodyB->GetName();
      contacts->shape_id[0] = fixtureA->GetUserData().pointer;
      contacts->shape_id[1] = fixtureB->GetUserData().pointer;
      contacts->transforms[0] = xfA;
      contacts->transforms[1] = xfB;
      contacts->normal = manifold->normal;
      contacts->separation = manifold->separation;
      contacts->points[0] = manifold->point - manifold->separation * manifold->normal;
      contacts->points[1] = manifold->point;
      contacts->local_points[0] = b2MulT(xfA, contacts->points[0]);
      contacts->local_points[1] = b2MulT(xfB, contacts->points[1]);
    }
  }
  return !collision;
}

/// Self collision test.
struct b2BVHManagerActiveDistanceWrapper
{
  bool DistanceCallback(void * proxyUserDataA, void * proxyUserDataB, b2Scalar & dist)
  {
    b2FixtureProxy * proxyA = (b2FixtureProxy *)proxyUserDataA;
    b2FixtureProxy * proxyB = (b2FixtureProxy *)proxyUserDataB;
    bool separation = DistanceResult(proxyA, proxyB, manager, manifold, contacts);
    if (manifold->separation < dist)
      dist = manifold->separation;
    return separation;
  }
  b2BVHManager * manager;
  b2Manifold * manifold{nullptr};
  b2ContactResult * contacts{nullptr};
};

bool b2BVHManager::Distance(b2Scalar & dist, b2Manifold * worldManifold)
{
  b2BVHManagerActiveDistanceWrapper wrapper;
  wrapper.manager = this;
  wrapper.manifold = worldManifold;
  if (!worldManifold) {
    b2Manifold manifold;
    wrapper.manifold = &manifold;
  }
  return m_broadPhase->Distance(&wrapper, dist);
}

bool b2BVHManager::Distance(b2Scalar & dist, b2ContactResult * contacts)
{
  b2BVHManagerActiveDistanceWrapper wrapper;
  wrapper.manager = this;
  wrapper.contacts = contacts;
  b2Manifold manifold;
  wrapper.manifold = &manifold;
  return m_broadPhase->Distance(&wrapper, dist);
}
