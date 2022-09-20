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

#ifndef B2_DYNAMIC_TREE_H
#define B2_DYNAMIC_TREE_H

#include "b2_api.h"
#include "b2_collision.h"
#include <stack>

#define b2_nullNode (-1)

/// A node in the dynamic tree. The client does not interact with this directly.
struct B2_API b2TreeNode
{
    B2_FORCE_INLINE bool IsLeaf() const
    {
        return child1 == b2_nullNode;
    }

    /// Enlarged AABB
    b2AABB aabb;

    void* userData;

    union
    {
        int32 parent;
        int32 next;
    };

    int32 child1;
    int32 child2;

    // leaf = 0, free node = -1
    int32 height;

    bool moved;
    bool extended;
};

/// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
/// A dynamic tree arranges data in a binary tree to accelerate
/// queries such as volume queries and ray casts. Leafs are proxies
/// with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
/// so that the proxy AABB is bigger than the client object. This allows the client
/// object to move by small amounts without triggering a tree update.
///
/// Nodes are pooled and relocatable, so we use node indices rather than pointers.
class B2_API b2DynamicTree
{
public:
    /// Constructing the tree initializes the node pool.
    b2DynamicTree();

    /// Destroy the tree, freeing the node pool.
    ~b2DynamicTree();

    /// Create a proxy. Provide a tight fitting AABB and a userData pointer.
    int32 CreateProxy(const b2AABB& aabb, void* userData, bool extend = true);

    bool UpdateProxy(int32 proxyId, bool extend);

    /// Destroy a proxy. This asserts if the id is invalid.
    void DestroyProxy(int32 proxyId);

    /// Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB,
    /// then the proxy is removed from the tree and re-inserted. Otherwise
    /// the function returns immediately.
    /// @return true if the proxy was re-inserted.
    bool MoveProxy(int32 proxyId, const b2AABB& aabb1, const b2Vec2& displacement);

    /// Update a proxy with a new AABB. If the proxy has moved outside of its fattened AABB,
    /// then the proxy is removed from the tree and re-inserted. Otherwise
    /// the function returns immediately.
    /// @return true if the proxy was re-inserted.
    bool UpdateProxy(int32 proxyId, const b2AABB& aabb);

    /// Get proxy user data.
    /// @return the proxy user data or 0 if the id is invalid.
    void* GetUserData(int32 proxyId) const;

    bool WasMoved(int32 proxyId) const;
    void ClearMoved(int32 proxyId);

    /// Get the fat AABB for a proxy.
    const b2AABB& GetFatAABB(int32 proxyId) const;

    /// Query an AABB for overlapping proxies. The callback class
    /// is called for each proxy that overlaps the supplied AABB.
    template <typename T>
    void Query(T* callback, const b2AABB& aabb) const;

    /// Query a point for overlapping proxies. The callback class
    /// is called for each proxy that contains the supplied point.
    template <typename T>
    void Query(T* callback, const b2Vec2& point) const;

    template <typename T>
    void Query(T* callback, const b2DynamicTree* tree) const;

    /// Ray-cast against the proxies in the tree. This relies on the callback
    /// to perform a exact ray-cast in the case were the proxy contains a shape.
    /// The callback also performs the any collision filtering. This has performance
    /// roughly equal to k * log(n), where k is the number of collisions and n is the
    /// number of proxies in the tree.
    /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
    /// @param callback a callback class that is called for each proxy that is hit by the ray.
    template <typename T>
    void RayCast(T* callback, const b2RayCastInput& input) const;

    /// Validate this tree. For testing.
    void Validate() const;

    int32 GetRoot() const;

    b2TreeNode* GetNode(int32 nodeId) const;

    /// Compute the height of the binary tree in O(N) time. Should not be
    /// called often.
    int32 GetHeight() const;

    /// Get the maximum balance of an node in the tree. The balance is the difference
    /// in height of the two children of a node.
    int32 GetMaxBalance() const;

    /// Get the ratio of the sum of the node areas to the root area.
    b2Scalar GetAreaRatio() const;

    /// Build an optimal tree. Very expensive. For testing.
    void RebuildBottomUp();

    /// Shift the world origin. Useful for large worlds.
    /// The shift formula is: position -= newOrigin
    /// @param newOrigin the new origin with respect to the old origin
    void ShiftOrigin(const b2Vec2& newOrigin);

    void setContactDistanceThreshold(b2Scalar contact_distance);

    b2Scalar getContactDistanceThreshold() const;

private:

    int32 AllocateNode();
    void FreeNode(int32 node);

    void InsertLeaf(int32 node);
    void RemoveLeaf(int32 node);

    int32 Balance(int32 index);

    int32 ComputeHeight() const;
    int32 ComputeHeight(int32 nodeId) const;

    void ValidateStructure(int32 index) const;
    void ValidateMetrics(int32 index) const;

    int32 m_root;

    b2TreeNode* m_nodes;
    int32 m_nodeCount;
    int32 m_nodeCapacity;

    int32 m_freeList;

    int32 m_insertionCount;

    b2Scalar m_contact_distance{0.0};
};

B2_FORCE_INLINE void* b2DynamicTree::GetUserData(int32 proxyId) const
{
//    b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
    return m_nodes[proxyId].userData;
}

B2_FORCE_INLINE bool b2DynamicTree::WasMoved(int32 proxyId) const
{
//    b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
    return m_nodes[proxyId].moved;
}

B2_FORCE_INLINE void b2DynamicTree::ClearMoved(int32 proxyId)
{
//    b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
    m_nodes[proxyId].moved = false;
}

B2_FORCE_INLINE const b2AABB& b2DynamicTree::GetFatAABB(int32 proxyId) const
{
//    b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
    return m_nodes[proxyId].aabb;
}

B2_FORCE_INLINE int32 b2DynamicTree::GetRoot() const
{
    return m_root;
}

B2_FORCE_INLINE b2TreeNode* b2DynamicTree::GetNode(int32 nodeId) const
{
    return m_nodes + nodeId;
}

template <typename T>
B2_FORCE_INLINE void b2DynamicTree::Query(T* callback, const b2AABB& aabb) const
{
    std::stack<int32> stack;
    stack.push(m_root);
    while (!stack.empty())
    {
        int32 nodeId = stack.top();
        stack.pop();
        if (nodeId == b2_nullNode)
            continue;

        const b2TreeNode* node = m_nodes + nodeId;
        if (b2TestOverlap(node->aabb, aabb))
        {
            if (node->IsLeaf())
            {
                if (!callback->QueryCallback(nodeId))
                    return;
            }
            else
            {
                stack.push(node->child1);
                stack.push(node->child2);
            }
        }
    }
}

template <typename T>
B2_FORCE_INLINE	void b2DynamicTree::Query(T* callback, const b2Vec2& point) const
{
    std::stack<int32> stack;
    stack.push(m_root);
    while (!stack.empty())
    {
        int32 nodeId = stack.top();
        stack.pop();
        if (nodeId == b2_nullNode)
            continue;

        const b2TreeNode* node = m_nodes + nodeId;
        if (node->aabb.Contains(point))
        {
            if (node->IsLeaf())
            {
                if (!callback->QueryCallback(nodeId))
                    return;
            }
            else
            {
                stack.push(node->child1);
                stack.push(node->child2);
            }
        }
    }
}

template <typename T>
B2_FORCE_INLINE void b2DynamicTree::Query(T* callback, const b2DynamicTree* tree) const
{
    std::stack<std::pair<int32, int32>> stack;
    stack.emplace(m_root, tree->GetRoot());
    while (!stack.empty())
    {
        std::pair<int32, int32> pair = stack.top();
        stack.pop();
        int32 nodeId1 = pair.first, nodeId2 = pair.second;
        if (nodeId1 == b2_nullNode || nodeId2 == b2_nullNode)
            continue;
        const b2TreeNode* node1 = GetNode(nodeId1);
        const b2TreeNode* node2 = tree->GetNode(nodeId2);
        if (b2TestOverlap(node1->aabb, node2->aabb))
        {
            if (node1->IsLeaf() && node2->IsLeaf())
            {
                if (!callback->QueryCallback(nodeId1, nodeId2))
                    return;
            }
            else if (node1->IsLeaf())
            {
                stack.emplace(nodeId1, node2->child1);
                stack.emplace(nodeId1, node2->child2);
            }
            else if (node2->IsLeaf())
            {
                stack.emplace(node1->child1, nodeId2);
                stack.emplace(node1->child2, nodeId2);
            }
            else 
            {
                stack.emplace(node1->child1, node2->child1);
                stack.emplace(node1->child1, node2->child2);
                stack.emplace(node1->child2, node2->child1);
                stack.emplace(node1->child2, node2->child2);
            }
        }
    }
}

template <typename T>
B2_FORCE_INLINE void b2DynamicTree::RayCast(T* callback, const b2RayCastInput& input) const
{
    b2Vec2 p1 = input.p1;
    b2Vec2 p2 = input.p2;
    b2Vec2 r = p2 - p1;
    b2Assert(r.LengthSquared() > b2Scalar(0.0));
    r.Normalize();

    // v is perpendicular to the segment.
    b2Vec2 v = b2Cross(b2Scalar(1.0), r);
    b2Vec2 abs_v = b2Abs(v);

    // Separating axis for segment (Gino, p80).
    // |dot(v, p1 - c)| > dot(|v|, h)

    b2Scalar maxFraction = input.maxFraction;

    // Build a bounding box for the segment.
    b2AABB segmentAABB;
    {
        b2Vec2 t = p1 + maxFraction * (p2 - p1);
        segmentAABB.lowerBound = b2Min(p1, t);
        segmentAABB.upperBound = b2Max(p1, t);
    }

    std::stack<int32> stack;
    stack.push(m_root);
    while (!stack.empty())
    {
        int32 nodeId = stack.top();
        stack.pop();
        if (nodeId == b2_nullNode)
        {
            continue;
        }

        const b2TreeNode* node = m_nodes + nodeId;

        if (b2TestOverlap(node->aabb, segmentAABB) == false)
        {
            continue;
        }

        // Separating axis for segment (Gino, p80).
        // |dot(v, p1 - c)| > dot(|v|, h)
        b2Vec2 c = node->aabb.GetCenter();
        b2Vec2 h = node->aabb.GetExtents();
        b2Scalar separation = b2Abs(b2Dot(v, p1 - c)) - b2Dot(abs_v, h);
        if (separation > b2Scalar(0.0))
        {
            continue;
        }

        if (node->IsLeaf())
        {
            b2RayCastInput subInput;
            subInput.p1 = input.p1;
            subInput.p2 = input.p2;
            subInput.maxFraction = maxFraction;

            b2Scalar value = callback->RayCastCallback(subInput, nodeId);

            if (value == b2Scalar(0.0))
            {
                // The client has terminated the ray cast.
                return;
            }

            if (value > b2Scalar(0.0))
            {
                // Update segment bounding box.
                maxFraction = value;
                b2Vec2 t = p1 + maxFraction * (p2 - p1);
                segmentAABB.lowerBound = b2Min(p1, t);
                segmentAABB.upperBound = b2Max(p1, t);
            }
        }
        else
        {
            stack.push(node->child1);
            stack.push(node->child2);
        }
    }
}

B2_FORCE_INLINE void b2DynamicTree::setContactDistanceThreshold(b2Scalar contact_distance)
{
    m_contact_distance = contact_distance;
}

B2_FORCE_INLINE b2Scalar b2DynamicTree::getContactDistanceThreshold() const
{
    return m_contact_distance;
}

#endif
