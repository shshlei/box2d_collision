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

#ifndef B2_BVH_MANAGER_H
#define B2_BVH_MANAGER_H

#include "b2_block_allocator.h"
#include "b2_stack_allocator.h"
#include "b2_broad_phase.h"
#include "b2_body.h"
#include "b2_fixture.h"
#include "b2_callbacks.h"

class B2_API b2BVHManager
{
public:
    /// Construct a BVHManager object.
    b2BVHManager();

    /// Destruct the BVHManager. All physics entities are destroyed and all heap memory is released.
    virtual ~b2BVHManager();

    b2Body* AddBody(const std::string& name, const b2Shape *shape, const b2Transform& xf = b2Transform(), bool active = true);

    b2Body* AddBody(const std::string& name, const std::vector<const b2Shape*>& shapes, const std::vector<b2Transform>& xfs, bool active = true);

    void AddShapeToBody(b2Body *body, const b2Shape *shape, const b2Transform& xf = b2Transform(), unsigned int shape_index = 0);

    b2Body* CreateBody(const std::string& name, bool active = true);

    /// Create a rigid body given a definition. No reference to the definition
    /// is retained.
    /// @warning This function is locked during callbacks.
    b2Body* CreateBody(const b2BodyDef* def);

    /// Destroy a rigid body given a definition. No reference to the definition
    /// is retained. This function is locked during callbacks.
    /// @warning This automatically deletes all associated shapes and joints.
    /// @warning This function is locked during callbacks.
    void DestroyBody(b2Body* body);

    b2Body* GetBody(const std::string& name);

    const b2Body* GetBody(const std::string& name) const;

    /**@brief Find if a collision object already exists
     * @param name The name of the collision object
     * @return true if it exists, otherwise false. */
    bool HasBody(const std::string& name) const;

    /**@brief Remove an object from the checker
     * @param name The name of the object
     * @return true if successfully removed, otherwise false. */
    bool RemoveBody(const std::string& name);

    /**@brief Enable an object
     * @param name The name of the object
     * @return true if successfully enabled, otherwise false. */
    bool EnableBody(const std::string& name);

    void EnableAll();

    /**@brief Disable an object
     * @param name The name of the object
     * @return true if successfully disabled, otherwise false. */
    bool DisableBody(const std::string& name);

    void DisableAll();

    bool IsBodyEnabled(const std::string& name) const;

    bool IsBodyActive(const std::string& name) const;

    void SetBodyActive(const std::string& name, bool active);

    /**@brief Set which collision objects are active
     * @param names A vector of collision object names */
    void SetActiveBodys(const std::vector<std::string>& names);

    /**@brief Set a single static collision object's tansform
     * @param name The name of the object
     * @param pose The tranformation in world */
    void SetBodyTransform(const std::string& name, const b2Transform& xf);

    void SetFilterData(const std::string& name, const b2Filter& filter);

    void SetUserData(const std::string& name, const b2BodyUserData& userData);

    /// Register a contact filter to provide specific control over collision.
    /// Otherwise the default filter is used (b2_defaultFilter). The listener is
    /// owned by you and must remain in scope.
    void SetContactFilter(b2ContactFilter* filter);

    b2ContactFilter* GetContactFilter();

    const b2ContactFilter* GetContactFilter() const;

    // return collision
    bool Collide(b2NaiveCallback* callback, const b2AABB& aabb) const;

    bool Collide(b2NaiveCallback* callback, const b2Vec2& point) const;

    bool Collide(b2NaiveCallback* callback, const b2BVHManager* manager) const;

    // return separation 
    bool Distance(b2NaiveCallback* callback, const b2AABB& aabb, b2Scalar &dist) const;

    bool Distance(b2NaiveCallback* callback, const b2Vec2& point, b2Scalar &dist) const;

    bool Distance(b2NaiveCallback* callback, const b2BVHManager* manager, b2Scalar &dist) const;

    bool SelfDistance(b2NaiveCallback* callback, b2Scalar &dist) const;
    
    // Active collision or separation
    /**@brief Perform a contact test for all objects
     * @param collisions The Contact results data */
    bool ContactTest(b2Manifold* worldManifold, b2InscribedSpheres* inscribedSpheres = nullptr);
    bool ContactTest(b2ContactResult* contacts = nullptr, b2InscribedSpheres* inscribedSpheres = nullptr);

    bool ContactTest(b2BVHManager *manager, b2Manifold* worldManifold, b2InscribedSpheres* inscribedSpheres = nullptr);
    bool ContactTest(b2BVHManager *manager, b2ContactResult* contacts = nullptr, b2InscribedSpheres* inscribedSpheres = nullptr);

    /**@brief Perform a distance test for all objects
     * @param collisions The Contact results data */
    bool DistanceTest(b2Scalar &dist, b2Manifold* worldManifold = nullptr);
    bool DistanceTest(b2Scalar &dist, b2ContactResult* contacts);

    /// Get the BVHManager body list. With the returned body, use b2Body::GetNext to get
    /// the next body in the BVHManager list. A nullptr body indicates the end of the list.
    /// @return the head of the BVHManager body list.
    b2Body* GetBodyList();

    const b2Body* GetBodyList() const;

    const b2BroadPhase* GetBroadPhase() const;

    /// Get the number of broad-phase proxies.
    int GetProxyCount() const;

    /// Get the number of bodies.
    int GetBodyCount() const;

    /// Get the height of the dynamic tree.
    int GetTreeHeight() const;

    /// Get the balance of the dynamic tree.
    int GetTreeBalance() const;

    /// Get the quality metric of the dynamic tree. The smaller the better.
    /// The minimum is 1.
    b2Scalar GetTreeQuality() const;

    /// Shift the BVHManager origin. Useful for large BVHManagers.
    /// The body shift formula is: position -= newOrigin
    /// @param newOrigin the new origin with respect to the old origin
    void ShiftOrigin(const b2Vec2& newOrigin);

    void setContactDistanceThreshold(b2Scalar contact_distance);

    b2Scalar getContactDistanceThreshold() const;

private:

    friend class b2Body;
    friend class b2Fixture;

    b2BlockAllocator m_blockAllocator;
    b2StackAllocator m_stackAllocator;

    b2BroadPhase *m_broadPhase;
    b2ContactFilter* m_contactFilter;

    b2Body* m_bodyList;
    int m_bodyCount;
};

B2_FORCE_INLINE void b2BVHManager::SetContactFilter(b2ContactFilter* filter)
{
    m_contactFilter = filter;
}

B2_FORCE_INLINE const b2ContactFilter* b2BVHManager::GetContactFilter() const
{
    return m_contactFilter;
}

B2_FORCE_INLINE b2ContactFilter* b2BVHManager::GetContactFilter()
{
    return m_contactFilter;
}

B2_FORCE_INLINE b2Body* b2BVHManager::GetBodyList()
{
    return m_bodyList;
}

B2_FORCE_INLINE const b2Body* b2BVHManager::GetBodyList() const
{
    return m_bodyList;
}

B2_FORCE_INLINE int b2BVHManager::GetBodyCount() const
{
    return m_bodyCount;
}

B2_FORCE_INLINE const b2BroadPhase* b2BVHManager::GetBroadPhase() const
{
    return m_broadPhase;
}

B2_FORCE_INLINE int b2BVHManager::GetProxyCount() const
{
    return m_broadPhase->GetProxyCount();
}

B2_FORCE_INLINE int b2BVHManager::GetTreeHeight() const
{
    return m_broadPhase->GetTreeHeight();
}

B2_FORCE_INLINE int b2BVHManager::GetTreeBalance() const
{
    return m_broadPhase->GetTreeBalance();
}

B2_FORCE_INLINE b2Scalar b2BVHManager::GetTreeQuality() const
{
    return m_broadPhase->GetTreeQuality();
}

B2_FORCE_INLINE void b2BVHManager::ShiftOrigin(const b2Vec2& newOrigin)
{
    for (b2Body* b = m_bodyList; b; b = b->m_next)
    {
        b->m_xf.p -= newOrigin;
    }

    m_broadPhase->ShiftOrigin(newOrigin);
}

B2_FORCE_INLINE void b2BVHManager::setContactDistanceThreshold(b2Scalar contact_distance)
{
    m_broadPhase->setContactDistanceThreshold(contact_distance);
}

B2_FORCE_INLINE b2Scalar b2BVHManager::getContactDistanceThreshold() const
{
    return m_broadPhase->getContactDistanceThreshold();
}

#endif
