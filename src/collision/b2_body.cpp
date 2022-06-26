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

#include "box2d_collision/b2_body.h"
#include "box2d_collision/b2_contact.h"
#include "box2d_collision/b2_fixture.h"

#include <new>

b2Body::b2Body(const b2BodyDef* bd)
{
//    b2Assert(bd->position.IsValid());
//    b2Assert(b2IsValid(bd->angle));

    m_name = bd->name;

    m_flags = 0;

    if (bd->bullet)
    {
        m_flags |= e_bulletFlag;
    }
    if (bd->fixedRotation)
    {
        m_flags |= e_fixedRotationFlag;
    }
    if (bd->allowSleep)
    {
        m_flags |= e_autoSleepFlag;
    }
    if (bd->awake && bd->type != b2_staticBody)
    {
        m_flags |= e_awakeFlag;
    }
    if (bd->enabled)
    {
        m_flags |= e_enabledFlag;
    }

    m_xf.p = bd->position;
    m_xf.q.Set(bd->angle);

    m_sweep.localCenter.SetZero();
    m_sweep.c0 = m_xf.p;
    m_sweep.c = m_xf.p;
    m_sweep.a0 = bd->angle;
    m_sweep.a = bd->angle;
    m_sweep.alpha0 = b2Scalar(0.0);

    m_prev = nullptr;
    m_next = nullptr;

    m_sleepTime = b2Scalar(0.0);

    m_type = bd->type;

    m_userData = bd->userData;

    m_fixtureList = nullptr;
    m_fixtureCount = 0;
}

b2Body::~b2Body()
{
    // shapes and joints are destroyed in b2World::Destroy
}

b2Fixture* b2Body::CreateFixture(const b2FixtureDef* def) 
{
    void* memory = m_blockAllocator->Allocate(sizeof(b2Fixture));
    b2Fixture* fixture = new (memory) b2Fixture;
    fixture->Create(m_blockAllocator, this, def);

    fixture->m_next = m_fixtureList;
    m_fixtureList = fixture;
    ++m_fixtureCount;

    fixture->m_body = this;

    return fixture;
}

b2Fixture* b2Body::CreateFixture(const b2Shape* shape, b2Scalar density, uint32 shape_index)
{
    b2FixtureDef def;
    def.shape = shape;
    def.density = density;
    def.userData.pointer = shape_index;
    return CreateFixture(&def);
}

b2Fixture* b2Body::AddShape(const b2Shape* shape, uint32 shape_index)
{
    b2FixtureDef def;
    def.shape = shape;
    def.userData.pointer = shape_index;
    return CreateFixture(&def);
}

void b2Body::DestroyFixture(b2Fixture* fixture)
{
    if (fixture == NULL)
    {
        return;
    }

//    b2Assert(fixture->m_body == this);

    // Remove the fixture from this body's singly linked list.
//    b2Assert(m_fixtureCount > 0);
    b2Fixture** node = &m_fixtureList;
    bool found = false;
    while (*node != nullptr)
    {
        if (*node == fixture)
        {
            *node = fixture->m_next;
            found = true;
            break;
        }

        node = &(*node)->m_next;
    }

    // You tried to remove a shape that is not attached to this body.
//    b2Assert(found);

    fixture->m_body = nullptr;
    fixture->m_next = nullptr;
    fixture->Destroy(m_blockAllocator);
    fixture->~b2Fixture();
    m_blockAllocator->Free(fixture, sizeof(b2Fixture));

    --m_fixtureCount;
}

void b2Body::Dump()
{
    int32 bodyIndex = m_islandIndex;

    // %.9g is sufficient to save and load the same value using text
    // FLT_DECIMAL_DIG == 9

    b2Dump("{\n");
    b2Dump("  b2BodyDef bd;\n");
    b2Dump("  bd.type = b2BodyType(%d);\n", m_type);
    b2Dump("  bd.position.Set(%.9g, %.9g);\n", m_xf.p.x, m_xf.p.y);
    b2Dump("  bd.angle = %.9g;\n", m_sweep.a);
    b2Dump("  bd.allowSleep = bool(%d);\n", m_flags & e_autoSleepFlag);
    b2Dump("  bd.awake = bool(%d);\n", m_flags & e_awakeFlag);
    b2Dump("  bd.fixedRotation = bool(%d);\n", m_flags & e_fixedRotationFlag);
    b2Dump("  bd.bullet = bool(%d);\n", m_flags & e_bulletFlag);
    b2Dump("  bd.enabled = bool(%d);\n", m_flags & e_enabledFlag);
    b2Dump("\n");
    for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
    {
        b2Dump("  {\n");
        f->Dump(bodyIndex);
        b2Dump("  }\n");
    }
    b2Dump("}\n");
}
