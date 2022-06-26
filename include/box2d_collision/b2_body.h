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

#ifndef B2_BODY_H
#define B2_BODY_H

#include "b2_api.h"
#include "b2_math.h"
#include "b2_shape.h"
#include "b2_block_allocator.h"

#include <string>

class b2Fixture;
class b2Contact;
struct b2FixtureDef;

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
enum b2BodyType
{
    b2_staticBody = 0,
    b2_kinematicBody,
    b2_dynamicBody
};

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
struct B2_API b2BodyDef
{
    /// This constructor sets the body definition default values.
    b2BodyDef(const std::string &bname)
    {
        name = bname;
        type = b2_staticBody;
        position.Set(b2Scalar(0.0), b2Scalar(0.0));
        angle = b2Scalar(0.0);
        allowSleep = true;
        awake = true;
        fixedRotation = false;
        bullet = false;
        enabled = true;
    }

    std::string name;

    /// The body type: static, kinematic, or dynamic.
    /// Note: if a dynamic body would have zero mass, the mass is set to one.
    b2BodyType type;

    /// The world position of the body. Avoid creating bodies at the origin
    /// since this can lead to many overlapping shapes.
    b2Vec2 position;

    /// The world angle of the body in radians.
    b2Scalar angle;

    /// Set this flag to false if this body should never fall asleep. Note that
    /// this increases CPU usage.
    bool allowSleep;

    /// Is this body initially awake or sleeping?
    bool awake;

    /// Should this body be prevented from rotating? Useful for characters.
    bool fixedRotation;

    /// Is this a fast moving body that should be prevented from tunneling through
    /// other moving bodies? Note that all bodies are prevented from tunneling through
    /// kinematic and static bodies. This setting is only considered on dynamic bodies.
    /// @warning You should use this flag sparingly since it increases processing time.
    bool bullet;

    /// Does this body start out enabled?
    bool enabled;

    /// Use this to store application specific body data.
    b2BodyUserData userData;
};

/// A rigid body. These are created via b2World::CreateBody.
class B2_API b2Body
{
public:

    b2Body(const b2BodyDef* bd);

    ~b2Body();

    void SetName(const std::string& name) 
    {
        m_name = name;
    }

    const std::string& GetName() const 
    {
        return m_name;
    }

    void SetBlockAllocator(b2BlockAllocator *blockAllocator)
    {
        m_blockAllocator = blockAllocator;
    }

    /// Creates a fixture and attach it to this body. Use this function if you need
    /// to set some fixture parameters, like friction. Otherwise you can create the
    /// fixture directly from a shape.
    /// If the density is non-zero, this function automatically updates the mass of the body.
    /// Contacts are not created until the next time step.
    /// @param def the fixture definition.
    /// @warning This function is locked during callbacks.
    b2Fixture* CreateFixture(const b2FixtureDef* def);

    /// Creates a fixture from a shape and attach it to this body.
    /// This is a convenience function. Use b2FixtureDef if you need to set parameters
    /// like friction, restitution, user data, or filtering.
    /// If the density is non-zero, this function automatically updates the mass of the body.
    /// @param shape the shape to be cloned.
    /// @param density the shape density (set to zero for static bodies).
    /// @warning This function is locked during callbacks.
    b2Fixture* CreateFixture(const b2Shape* shape, b2Scalar density, uint32 shape_index = 0);

    b2Fixture* AddShape(const b2Shape* shape, uint32 shape_index = 0);

    /// Destroy a fixture. This removes the fixture from the broad-phase and
    /// destroys all contacts associated with this fixture. This will
    /// automatically adjust the mass of the body if the body is dynamic and the
    /// fixture has positive density.
    /// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
    /// @param fixture the fixture to be removed.
    /// @warning This function is locked during callbacks.
    void DestroyFixture(b2Fixture* fixture);

    /// Set the position of the body's origin and rotation.
    /// Manipulating a body's transform may cause non-physical behavior.
    /// Note: contacts are updated on the next call to b2World::Step.
    /// @param position the world position of the body's local origin.
    /// @param angle the world rotation in radians.
    void SetTransform(const b2Vec2& position, b2Scalar angle);

    void SetTransform(const b2Vec2& position1, b2Scalar angle1, const b2Vec2& position2, b2Scalar angle2);

    void SetTransform(const b2Transform& xf);

    void SetTransform(const b2Transform& xf1, const b2Transform& xf2);

    /// Get the body transform for the body's origin.
    /// @return the world transform of the body's origin.
    const b2Transform& GetTransform() const;

    /// Get the world body origin position.
    /// @return the world position of the body's origin.
    const b2Vec2& GetPosition() const;

    /// Get the angle in radians.
    /// @return the current world rotation angle in radians.
    b2Scalar GetAngle() const;

    /// Get the world position of the center of mass.
    const b2Vec2& GetWorldCenter() const;

    /// Get the local position of the center of mass.
    const b2Vec2& GetLocalCenter() const;

    /// Get the world coordinates of a point given the local coordinates.
    /// @param localPoint a point on the body measured relative the the body's origin.
    /// @return the same point expressed in world coordinates.
    b2Vec2 GetWorldPoint(const b2Vec2& localPoint) const;

    /// Get the world coordinates of a vector given the local coordinates.
    /// @param localVector a vector fixed in the body.
    /// @return the same vector expressed in world coordinates.
    b2Vec2 GetWorldVector(const b2Vec2& localVector) const;

    /// Gets a local point relative to the body's origin given a world point.
    /// @param worldPoint a point in world coordinates.
    /// @return the corresponding local point relative to the body's origin.
    b2Vec2 GetLocalPoint(const b2Vec2& worldPoint) const;

    /// Gets a local vector given a world vector.
    /// @param worldVector a vector in world coordinates.
    /// @return the corresponding local vector.
    b2Vec2 GetLocalVector(const b2Vec2& worldVector) const;

    /// Set the type of this body. This may alter the mass and velocity.
    void SetType(b2BodyType type);

    /// Get the type of this body.
    b2BodyType GetType() const;

    /// Should this body be treated like a bullet for continuous collision detection?
    void SetBullet(bool flag);

    /// Is this body treated like a bullet for continuous collision detection?
    bool IsBullet() const;

    /// You can disable sleeping on this body. If you disable sleeping, the
    /// body will be woken.
    void SetSleepingAllowed(bool flag);

    /// Is this body allowed to sleep
    bool IsSleepingAllowed() const;

    /// Set the sleep state of the body. A sleeping body has very
    /// low CPU cost.
    /// @param flag set to true to wake the body, false to put it to sleep.
    void SetAwake(bool flag);

    /// Get the sleeping state of this body.
    /// @return true if the body is awake.
    bool IsAwake() const;

    /// Allow a body to be disabled. A disabled body is not simulated and cannot
    /// be collided with or woken up.
    /// If you pass a flag of true, all fixtures will be added to the broad-phase.
    /// If you pass a flag of false, all fixtures will be removed from the
    /// broad-phase and all contacts will be destroyed.
    /// Fixtures and joints are otherwise unaffected. You may continue
    /// to create/destroy fixtures and joints on disabled bodies.
    /// Fixtures on a disabled body are implicitly disabled and will
    /// not participate in collisions, ray-casts, or queries.
    /// Joints connected to a disabled body are implicitly disabled.
    /// An diabled body is still owned by a b2World object and remains
    /// in the body list.
    void SetEnabled(bool flag);

    /// Get the active state of the body.
    bool IsEnabled() const;

    bool IsActive() const;

    /// Set this body to have fixed rotation. This causes the mass
    /// to be reset.
    void SetFixedRotation(bool flag);

    /// Does this body have fixed rotation?
    bool IsFixedRotation() const;

    /// Get the list of all fixtures attached to this body.
    b2Fixture* GetFixtureList();
    const b2Fixture* GetFixtureList() const;

    /// Get the next body in the world's body list.
    b2Body* GetNext();
    const b2Body* GetNext() const;

    void SetUserData(const b2BodyUserData& userData);

    /// Get the user data pointer that was provided in the body definition.
    b2BodyUserData& GetUserData();
    const b2BodyUserData& GetUserData() const;

    /// Dump this body to a file
    void Dump();

private:

    friend class b2Contact;
    friend class b2BVHManager;

    std::string m_name;

    // m_flags
    enum
    {
        e_islandFlag		= 0x0001,
        e_awakeFlag			= 0x0002,
        e_autoSleepFlag		= 0x0004,
        e_bulletFlag		= 0x0008,
        e_fixedRotationFlag	= 0x0010,
        e_enabledFlag		= 0x0020,
        e_toiFlag			= 0x0040
    };

    // This is used to prevent connected bodies from colliding.
    // It may lie, depending on the collideConnected flag.
    bool ShouldCollide(const b2Body* other) const;

    b2BlockAllocator *m_blockAllocator;

    b2BodyType m_type;

    uint16 m_flags;

    int32 m_islandIndex;

    b2Transform m_xf;		// the body origin transform
    b2Sweep m_sweep;		// the swept motion for CCD

    b2Body* m_prev;
    b2Body* m_next;

    b2Fixture* m_fixtureList;
    int32 m_fixtureCount;

    b2Scalar m_sleepTime;

    b2BodyUserData m_userData;
};

B2_FORCE_INLINE void b2Body::SetType(b2BodyType type)
{
    if (m_type == type)
    {
        return;
    }

    m_type = type;

    SetAwake(true);
}

B2_FORCE_INLINE b2BodyType b2Body::GetType() const
{
    return m_type;
}

B2_FORCE_INLINE void b2Body::SetTransform(const b2Vec2& position, b2Scalar angle)
{
    m_xf.q.Set(angle);
    m_xf.p = position;

    m_sweep.c = position;
    m_sweep.a = angle;

    m_sweep.c0 = m_sweep.c;
    m_sweep.a0 = angle;
}

B2_FORCE_INLINE void b2Body::SetTransform(const b2Vec2& position1, b2Scalar angle1, const b2Vec2& position2, b2Scalar angle2)
{
    m_xf.q.Set(angle1);
    m_xf.p = position1;

    m_sweep.c0 = position1;
    m_sweep.a0 = angle1;

    m_sweep.c = position2;
    m_sweep.a = angle2;
}

B2_FORCE_INLINE void b2Body::SetTransform(const b2Transform& xf)
{
    SetTransform(xf.p, b2Atan2(xf.q.s, xf.q.c));
}

B2_FORCE_INLINE void b2Body::SetTransform(const b2Transform& xf1, const b2Transform& xf2)
{
    SetTransform(xf1.p, b2Atan2(xf1.q.s, xf1.q.c), xf2.p, b2Atan2(xf2.q.s, xf2.q.c));
}

B2_FORCE_INLINE const b2Transform& b2Body::GetTransform() const
{
    return m_xf;
}

B2_FORCE_INLINE const b2Vec2& b2Body::GetPosition() const
{
    return m_xf.p;
}

B2_FORCE_INLINE b2Scalar b2Body::GetAngle() const
{
    return m_sweep.a0;
}

B2_FORCE_INLINE const b2Vec2& b2Body::GetWorldCenter() const
{
    return m_sweep.c0;
}

B2_FORCE_INLINE const b2Vec2& b2Body::GetLocalCenter() const
{
    return m_sweep.localCenter;
}

B2_FORCE_INLINE b2Vec2 b2Body::GetWorldPoint(const b2Vec2& localPoint) const
{
    return b2Mul(m_xf, localPoint);
}

B2_FORCE_INLINE b2Vec2 b2Body::GetWorldVector(const b2Vec2& localVector) const
{
    return b2Mul(m_xf.q, localVector);
}

B2_FORCE_INLINE b2Vec2 b2Body::GetLocalPoint(const b2Vec2& worldPoint) const
{
    return b2MulT(m_xf, worldPoint);
}

B2_FORCE_INLINE b2Vec2 b2Body::GetLocalVector(const b2Vec2& worldVector) const
{
    return b2MulT(m_xf.q, worldVector);
}

B2_FORCE_INLINE void b2Body::SetBullet(bool flag)
{
    if (flag)
    {
        m_flags |= e_bulletFlag;
    }
    else
    {
        m_flags &= ~e_bulletFlag;
    }
}

B2_FORCE_INLINE bool b2Body::IsBullet() const
{
    return (m_flags & e_bulletFlag) == e_bulletFlag;
}

B2_FORCE_INLINE void b2Body::SetAwake(bool flag)
{
    if (m_type == b2_staticBody)
    {
        return;
    }

    if (flag)
    {
        m_flags |= e_awakeFlag;
        m_sleepTime = b2Scalar(0.0);
    }
    else
    {
        m_flags &= ~e_awakeFlag;
        m_sleepTime = b2Scalar(0.0);
    }
}

B2_FORCE_INLINE bool b2Body::IsAwake() const
{
    return (m_flags & e_awakeFlag) == e_awakeFlag;
}

B2_FORCE_INLINE void b2Body::SetEnabled(bool flag)
{
    if (flag)
    {
        m_flags |= e_enabledFlag;
    }
    else
    {
        m_flags &= ~e_enabledFlag;
    }
}

B2_FORCE_INLINE bool b2Body::IsEnabled() const
{
    return (m_flags & e_enabledFlag) == e_enabledFlag;
}

B2_FORCE_INLINE bool b2Body::IsActive() const
{
    return m_type != b2_staticBody;
}

B2_FORCE_INLINE void b2Body::SetFixedRotation(bool flag)
{
    bool status = (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
    if (status == flag)
    {
        return;
    }

    if (flag)
    {
        m_flags |= e_fixedRotationFlag;
    }
    else
    {
        m_flags &= ~e_fixedRotationFlag;
    }
}

B2_FORCE_INLINE bool b2Body::IsFixedRotation() const
{
    return (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
}

B2_FORCE_INLINE void b2Body::SetSleepingAllowed(bool flag)
{
    if (flag)
    {
        m_flags |= e_autoSleepFlag;
    }
    else
    {
        m_flags &= ~e_autoSleepFlag;
        SetAwake(true);
    }
}

B2_FORCE_INLINE bool b2Body::IsSleepingAllowed() const
{
    return (m_flags & e_autoSleepFlag) == e_autoSleepFlag;
}

B2_FORCE_INLINE b2Fixture* b2Body::GetFixtureList()
{
    return m_fixtureList;
}

B2_FORCE_INLINE const b2Fixture* b2Body::GetFixtureList() const
{
    return m_fixtureList;
}

B2_FORCE_INLINE b2Body* b2Body::GetNext()
{
    return m_next;
}

B2_FORCE_INLINE const b2Body* b2Body::GetNext() const
{
    return m_next;
}

B2_FORCE_INLINE void b2Body::SetUserData(const b2BodyUserData& userData)
{
    m_userData = userData;
}

B2_FORCE_INLINE b2BodyUserData& b2Body::GetUserData()
{
    return m_userData;
}

B2_FORCE_INLINE const b2BodyUserData& b2Body::GetUserData() const
{
    return m_userData;
}

B2_FORCE_INLINE bool b2Body::ShouldCollide(const b2Body* other) const
{
    // At least one body should be dynamic.
    if (m_type != b2_dynamicBody && other->m_type != b2_dynamicBody)
    {
        return false;
    }

    return true;
}

#endif
