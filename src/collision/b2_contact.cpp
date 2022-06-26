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


#include "b2_contact_chain_circle.h"
#include "b2_contact_chain_polygon.h"
#include "b2_contact_circle.h"
#include "b2_contact_edge_circle.h"
#include "b2_contact_edge_polygon.h"
#include "b2_contact_polygon_circle.h"
#include "b2_contact_polygon.h"

#include "box2d_collision/b2_contact.h"
#include "box2d_collision/b2_block_allocator.h"
#include "box2d_collision/b2_body.h"
#include "box2d_collision/b2_collision.h"
#include "box2d_collision/b2_fixture.h"
#include "box2d_collision/b2_shape.h"
#include "box2d_collision/b2_time_of_impact.h"

b2ContactRegister b2Contact::s_registers[b2Shape::e_typeCount][b2Shape::e_typeCount];
bool b2Contact::s_initialized = false;

void b2Contact::InitializeRegisters()
{
    AddType(b2CircleContact::Create, b2CircleContact::Destroy, b2Shape::e_circle, b2Shape::e_circle);
    AddType(b2PolygonAndCircleContact::Create, b2PolygonAndCircleContact::Destroy, b2Shape::e_polygon, b2Shape::e_circle);
    AddType(b2PolygonContact::Create, b2PolygonContact::Destroy, b2Shape::e_polygon, b2Shape::e_polygon);
    AddType(b2EdgeAndCircleContact::Create, b2EdgeAndCircleContact::Destroy, b2Shape::e_edge, b2Shape::e_circle);
    AddType(b2EdgeAndPolygonContact::Create, b2EdgeAndPolygonContact::Destroy, b2Shape::e_edge, b2Shape::e_polygon);
    AddType(b2ChainAndCircleContact::Create, b2ChainAndCircleContact::Destroy, b2Shape::e_chain, b2Shape::e_circle);
    AddType(b2ChainAndPolygonContact::Create, b2ChainAndPolygonContact::Destroy, b2Shape::e_chain, b2Shape::e_polygon);
}

void b2Contact::AddType(b2ContactCreateFcn* createFcn, b2ContactDestroyFcn* destoryFcn,
        b2Shape::Type type1, b2Shape::Type type2)
{
//    b2Assert(0 <= type1 && type1 < b2Shape::e_typeCount);
//    b2Assert(0 <= type2 && type2 < b2Shape::e_typeCount);

    s_registers[type1][type2].createFcn = createFcn;
    s_registers[type1][type2].destroyFcn = destoryFcn;
    s_registers[type1][type2].primary = true;

    if (type1 != type2)
    {
        s_registers[type2][type1].createFcn = createFcn;
        s_registers[type2][type1].destroyFcn = destoryFcn;
        s_registers[type2][type1].primary = false;
    }
}

b2Contact* b2Contact::Create(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB, b2BlockAllocator* allocator)
{
    if (s_initialized == false)
    {
        InitializeRegisters();
        s_initialized = true;
    }

    b2Shape::Type type1 = fixtureA->GetType();
    b2Shape::Type type2 = fixtureB->GetType();

//    b2Assert(0 <= type1 && type1 < b2Shape::e_typeCount);
//    b2Assert(0 <= type2 && type2 < b2Shape::e_typeCount);

    b2ContactCreateFcn* createFcn = s_registers[type1][type2].createFcn;
    if (createFcn)
    {
        if (s_registers[type1][type2].primary)
        {
            return createFcn(fixtureA, indexA, fixtureB, indexB, allocator);
        }
        else
        {
            return createFcn(fixtureB, indexB, fixtureA, indexA, allocator);
        }
    }
    else
    {
        return nullptr;
    }
}

void b2Contact::Destroy(b2Contact* contact, b2BlockAllocator* allocator)
{
//    b2Assert(s_initialized == true);

    b2Fixture* fixtureA = contact->m_fixtureA;
    b2Fixture* fixtureB = contact->m_fixtureB;

    if (contact->m_manifold.pointCount > 0 &&
            fixtureA->IsSensor() == false &&
            fixtureB->IsSensor() == false)
    {
        fixtureA->GetBody()->SetAwake(true);
        fixtureB->GetBody()->SetAwake(true);
    }

    b2Shape::Type typeA = fixtureA->GetType();
    b2Shape::Type typeB = fixtureB->GetType();

//    b2Assert(0 <= typeA && typeA < b2Shape::e_typeCount);
//    b2Assert(0 <= typeB && typeB < b2Shape::e_typeCount);

    b2ContactDestroyFcn* destroyFcn = s_registers[typeA][typeB].destroyFcn;
    destroyFcn(contact, allocator);
}

b2Contact::b2Contact(b2Fixture* fA, int32 indexA, b2Fixture* fB, int32 indexB)
{
    m_flags = e_enabledFlag;

    m_fixtureA = fA;
    m_fixtureB = fB;

    m_indexA = indexA;
    m_indexB = indexB;

    m_manifold.pointCount = 0;

    m_prev = nullptr;
    m_next = nullptr;

    m_nodeA.contact = nullptr;
    m_nodeA.prev = nullptr;
    m_nodeA.next = nullptr;
    m_nodeA.other = nullptr;

    m_nodeB.contact = nullptr;
    m_nodeB.prev = nullptr;
    m_nodeB.next = nullptr;
    m_nodeB.other = nullptr;

    m_toiCount = 0;
}
