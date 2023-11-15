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

#include "b2_block_allocator.h"
#include "b2_shape.h"

#include <string>

class b2Fixture;
struct b2FixtureDef;

/// You can define this to inject whatever data you want in b2Body
struct B2_API b2BodyUserData
{
  b2BodyUserData()
  {
    pointer = 0;
  }

  /// For legacy compatibility
  uintptr_t pointer;
};

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
  b2BodyDef(const std::string & bname)
  {
    name = bname;
    type = b2_staticBody;
    position.setZero();
    angle = b2Scalar(0.0);
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

  /// Does this body start out enabled?
  bool enabled;

  /// Use this to store application specific body data.
  b2BodyUserData userData;
};

/// A rigid body. These are created via b2World::CreateBody.
class B2_API b2Body
{
public:
  b2Body(const b2BodyDef * bd);

  ~b2Body();

  void SetName(const std::string & name)
  {
    m_name = name;
  }

  const std::string & GetName() const
  {
    return m_name;
  }

  void SetBlockAllocator(b2BlockAllocator * blockAllocator)
  {
    m_blockAllocator = blockAllocator;
  }

  /// Creates a fixture and attach it to this body.
  /// @param def the fixture definition.
  /// @warning This function is locked during callbacks.
  b2Fixture * CreateFixture(const b2FixtureDef * def);

  /// Creates a fixture from a shape and attach it to this body.
  /// @param shape the shape to be cloned.
  /// @param xf the shape local transform
  /// @warning This function is locked during callbacks.
  b2Fixture * CreateFixture(const b2Shape * shape, const b2Transform & xf = b2Transform::Identity(), unsigned int shape_index = 0);

  b2Fixture * AddShape(const b2Shape * shape, const b2Transform & xf = b2Transform::Identity(), unsigned int shape_index = 0);

  /// Destroy a fixture. This removes the fixture from the broad-phase and
  /// destroys all contacts associated with this fixture. This will
  /// automatically adjust the mass of the body if the body is dynamic and the
  /// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
  /// @param fixture the fixture to be removed.
  /// @warning This function is locked during callbacks.
  void DestroyFixture(b2Fixture * fixture);

  /// Set the position of the body's origin and rotation.
  /// Manipulating a body's transform may cause non-physical behavior.
  /// Note: contacts are updated on the next call to b2World::Step.
  /// @param position the world position of the body's local origin.
  /// @param angle the world rotation in radians.
  void SetTransform(const b2Vec2 & position, b2Scalar angle);

  void SetTransform(const b2Transform & xf);

  /// Get the body transform for the body's origin.
  /// @return the world transform of the body's origin.
  const b2Transform & GetTransform() const;

  /// Get the world body origin position.
  /// @return the world position of the body's origin.
  b2Vec2 GetPosition() const;

  /// Get the angle in radians.
  /// @return the current world rotation angle in radians.
  b2Scalar GetAngle() const;

  /// Get the world position of the center of mass.
  b2Vec2 GetWorldCenter() const;

  /// Get the world coordinates of a point given the local coordinates.
  /// @param localPoint a point on the body measured relative the the body's origin.
  /// @return the same point expressed in world coordinates.
  b2Vec2 GetWorldPoint(const b2Vec2 & localPoint) const;

  /// Get the world coordinates of a vector given the local coordinates.
  /// @param localVector a vector fixed in the body.
  /// @return the same vector expressed in world coordinates.
  b2Vec2 GetWorldVector(const b2Vec2 & localVector) const;

  /// Gets a local point relative to the body's origin given a world point.
  /// @param worldPoint a point in world coordinates.
  /// @return the corresponding local point relative to the body's origin.
  b2Vec2 GetLocalPoint(const b2Vec2 & worldPoint) const;

  /// Gets a local vector given a world vector.
  /// @param worldVector a vector in world coordinates.
  /// @return the corresponding local vector.
  b2Vec2 GetLocalVector(const b2Vec2 & worldVector) const;

  /// Set the type of this body. This may alter the mass and velocity.
  void SetType(b2BodyType type);

  /// Get the type of this body.
  b2BodyType GetType() const;

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

  /// Get the list of all fixtures attached to this body.
  b2Fixture * GetFixtureList();
  const b2Fixture * GetFixtureList() const;

  /// Get the next body in the world's body list.
  b2Body * GetNext();
  const b2Body * GetNext() const;

  void SetUserData(const b2BodyUserData & userData);

  /// Get the user data pointer that was provided in the body definition.
  b2BodyUserData & GetUserData();
  const b2BodyUserData & GetUserData() const;

  // This is used to prevent connected bodies from colliding.
  // It may lie, depending on the collideConnected flag.
  bool ShouldCollide(const b2Body * other) const;

private:
  friend class b2BVHManager;

  std::string m_name;

  b2BlockAllocator * m_blockAllocator;

  b2BodyType m_type;

  b2Transform m_xf;  // the body origin transform
  b2Scalar m_angle;

  b2Body * m_prev;
  b2Body * m_next;

  b2Fixture * m_fixtureList;
  int m_fixtureCount;

  b2BodyUserData m_userData;

  /// Does this body start out enabled?
  bool m_enabled;
};

#endif
