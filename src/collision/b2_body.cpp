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
#include "box2d_collision/b2_fixture.h"

#include <new>

b2Body::b2Body(const b2BodyDef* bd)
{
    m_name = bd->name;

    m_xf.p = bd->position;
    m_xf.q.Set(bd->angle);
    m_angle = bd->angle;

    m_prev = nullptr;
    m_next = nullptr;

    m_type = bd->type;

    m_userData = bd->userData;

    m_fixtureList = nullptr;
    m_fixtureCount = 0;
    m_enabled = bd->enabled;
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

b2Fixture* b2Body::CreateFixture(const b2Shape* shape, const b2Transform& xf, unsigned int shape_index)
{
    b2FixtureDef def;
    def.shape = shape;
    def.xf = xf;
    def.userData.pointer = shape_index;
    return CreateFixture(&def);
}

b2Fixture* b2Body::AddShape(const b2Shape* shape, const b2Transform& xf, unsigned int shape_index)
{
    b2FixtureDef def;
    def.shape = shape;
    def.xf = xf;
    def.userData.pointer = shape_index;
    return CreateFixture(&def);
}

void b2Body::DestroyFixture(b2Fixture* fixture)
{
    if (fixture == NULL)
        return;
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
