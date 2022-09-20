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

#ifndef B2_CONTACT_H
#define B2_CONTACT_H

#include "b2_api.h"
#include "b2_collision.h"
#include "b2_fixture.h"
#include "b2_math.h"
#include "b2_shape.h"

class b2Body;
class b2Contact;
class b2Fixture;
class b2BlockAllocator;
class b2StackAllocator;

typedef b2Contact* b2ContactCreateFcn(b2Fixture* fixtureA, b2Fixture* fixtureB, b2BlockAllocator* allocator);
typedef void b2ContactDestroyFcn(b2Contact* contact, b2BlockAllocator* allocator);

struct B2_API b2ContactRegister
{
    b2ContactCreateFcn* createFcn;
    b2ContactDestroyFcn* destroyFcn;
    bool primary;
};

/// A contact edge is used to connect bodies and contacts together
/// in a contact graph where each body is a node and each contact
/// is an edge. A contact edge belongs to a doubly linked list
/// maintained in each attached body. Each contact has two contact
/// nodes, one for each attached body.
struct B2_API b2ContactEdge
{
    b2Body* other;			///< provides quick access to the other body attached.
    b2Contact* contact;		///< the contact
    b2ContactEdge* prev;	///< the previous contact edge in the body's contact list
    b2ContactEdge* next;	///< the next contact edge in the body's contact list
};

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
class B2_API b2Contact
{
public:

    /// Is this contact touching?
    bool IsTouching() const;

    /// Get the next contact in the world's contact list.
    b2Contact* GetNext();
    const b2Contact* GetNext() const;

    /// Get fixture A in this contact.
    b2Fixture* GetFixtureA();
    const b2Fixture* GetFixtureA() const;

    /// Get fixture B in this contact.
    b2Fixture* GetFixtureB();
    const b2Fixture* GetFixtureB() const;

    /// Evaluate this contact with your own manifold and transforms.
    virtual bool Evaluate(b2Manifold* manifold, const b2Transform& xfA, const b2Transform& xfB) = 0;

protected:

    friend class b2Body;
    friend class b2Fixture;
    friend class b2BVHManager;

    static void AddType(b2ContactCreateFcn* createFcn, b2ContactDestroyFcn* destroyFcn, b2Shape::Type typeA, b2Shape::Type typeB);
    static void InitializeRegisters();
    static b2Contact* Create(b2Fixture* fixtureA, b2Fixture* fixtureB, b2BlockAllocator* allocator);
    static void Destroy(b2Contact* contact, b2BlockAllocator* allocator);

    b2Contact() : m_fixtureA(nullptr), m_fixtureB(nullptr) {}
    b2Contact(b2Fixture* fixtureA, b2Fixture* fixtureB);
    virtual ~b2Contact() {}

    static b2ContactRegister s_registers[b2Shape::e_typeCount][b2Shape::e_typeCount];
    static bool s_initialized;

    // World pool and list pointers.
    b2Contact* m_prev;
    b2Contact* m_next;

    // Nodes for connecting bodies.
    b2ContactEdge m_nodeA;
    b2ContactEdge m_nodeB;

    b2Fixture* m_fixtureA;
    b2Fixture* m_fixtureB;

    int32 m_toiCount;
    b2Scalar m_toi;
};

B2_FORCE_INLINE b2Contact* b2Contact::GetNext()
{
    return m_next;
}

B2_FORCE_INLINE const b2Contact* b2Contact::GetNext() const
{
    return m_next;
}

B2_FORCE_INLINE b2Fixture* b2Contact::GetFixtureA()
{
    return m_fixtureA;
}

B2_FORCE_INLINE const b2Fixture* b2Contact::GetFixtureA() const
{
    return m_fixtureA;
}

B2_FORCE_INLINE b2Fixture* b2Contact::GetFixtureB()
{
    return m_fixtureB;
}

B2_FORCE_INLINE const b2Fixture* b2Contact::GetFixtureB() const
{
    return m_fixtureB;
}

#endif
