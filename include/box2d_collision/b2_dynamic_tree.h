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
#include "b2_math.h"

#include <stack>

#define b2_nullNode (-1)

/// A node in the dynamic tree. The client does not interact with this directly.
struct B2_API b2TreeNode
{
  bool IsLeaf() const
  {
    return child1 == b2_nullNode;
  }

  /// Enlarged AABB
  b2AABB aabb;

  void * userData;

  union
  {
    int parent;
    int next;
  };

  int child1;
  int child2;

  // leaf = 0, free node = -1
  int height;
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
  int CreateProxy(const b2AABB & aabb, void * userData);

  /// Destroy a proxy. This asserts if the id is invalid.
  void DestroyProxy(int proxyId);

  /// Update a proxy with a new AABB. If the proxy has moved outside of its fattened AABB,
  /// then the proxy is removed from the tree and re-inserted. Otherwise
  /// the function returns immediately.
  /// @return true if the proxy was re-inserted.
  void UpdateProxy(int proxyId, const b2AABB & aabb);

  /// Get proxy user data.
  /// @return the proxy user data or 0 if the id is invalid.
  void * GetUserData(int proxyId) const;

  /// Get the fat AABB for a proxy.
  const b2AABB & GetFatAABB(int proxyId) const;

  // Return collision
  template <typename T>
  bool Collide(T * callback, const b2AABB & aabb) const;

  template <typename T>
  bool Collide(T * callback, const b2Vec2 & point) const;

  template <typename T>
  bool Collide(T * callback, const b2DynamicTree * tree) const;

  // Return separation
  template <typename T>
  bool Distance(T * callback, const b2AABB & aabb, b2Scalar & dist) const;

  template <typename T>
  bool Distance(T * callback, const b2Vec2 & point, b2Scalar & dist) const;

  template <typename T>
  bool Distance(T * callback, const b2DynamicTree * tree, b2Scalar & dist) const;

  template <typename T>
  bool SelfDistance(T * callback, b2Scalar & dist) const;

  /// Validate this tree. For testing.
  void Validate() const;

  int GetRoot() const;

  const b2TreeNode * GetNode(int nodeId) const;

  /// Compute the height of the binary tree in O(N) time. Should not be
  /// called often.
  int GetHeight() const;

  /// Get the maximum balance of an node in the tree. The balance is the difference
  /// in height of the two children of a node.
  int GetMaxBalance() const;

  /// Get the ratio of the sum of the node areas to the root area.
  b2Scalar GetAreaRatio() const;

  /// Build an optimal tree. Very expensive. For testing.
  void RebuildBottomUp();

  /// Shift the world origin. Useful for large worlds.
  /// The shift formula is: position -= newOrigin
  /// @param newOrigin the new origin with respect to the old origin
  void ShiftOrigin(const b2Vec2 & newOrigin);

  void setContactDistanceThreshold(b2Scalar contact_distance);

  b2Scalar getContactDistanceThreshold() const;

private:
  int AllocateNode();
  void FreeNode(int node);

  void InsertLeaf(int node);
  void RemoveLeaf(int node);

  int Balance(int index);

  int ComputeHeight() const;
  int ComputeHeight(int nodeId) const;

  void ValidateStructure(int index) const;
  void ValidateMetrics(int index) const;

  int m_root;

  b2TreeNode * m_nodes;
  int m_nodeCount;
  int m_nodeCapacity;

  int m_freeList;

  b2Scalar m_contact_distance{0.0};
};

template <typename T>
bool b2DynamicTree::Collide(T * callback, const b2AABB & aabb) const
{
  std::stack<int> stack;
  stack.push(m_root);
  while (!stack.empty()) {
    int nodeId = stack.top();
    stack.pop();
    if (nodeId == b2_nullNode)
      continue;

    const b2TreeNode * node = m_nodes + nodeId;
    if (b2TestOverlap(node->aabb, aabb)) {
      if (node->IsLeaf()) {
        if (callback->CollideCallback(nodeId))  // could stop
          return true;
      }
      else {
        stack.push(node->child1);
        stack.push(node->child2);
      }
    }
  }
  return false;
}

template <typename T>
bool b2DynamicTree::Collide(T * callback, const b2Vec2 & point) const
{
  std::stack<int> stack;
  stack.push(m_root);
  while (!stack.empty()) {
    int nodeId = stack.top();
    stack.pop();
    if (nodeId == b2_nullNode)
      continue;

    const b2TreeNode * node = m_nodes + nodeId;
    if (node->aabb.contains(point)) {
      if (node->IsLeaf()) {
        if (callback->CollideCallback(nodeId))
          return true;
      }
      else {
        stack.push(node->child1);
        stack.push(node->child2);
      }
    }
  }
  return false;
}

template <typename T>
bool b2DynamicTree::Collide(T * callback, const b2DynamicTree * tree) const
{
  std::stack<std::pair<int, int>> stack;
  stack.emplace(m_root, tree->GetRoot());
  while (!stack.empty()) {
    const std::pair<int, int> & pair = stack.top();
    stack.pop();
    int nodeId1 = pair.first, nodeId2 = pair.second;
    if (nodeId1 == b2_nullNode || nodeId2 == b2_nullNode)
      continue;
    const b2TreeNode * node1 = GetNode(nodeId1);
    const b2TreeNode * node2 = tree->GetNode(nodeId2);
    if (b2TestOverlap(node1->aabb, node2->aabb)) {
      if (node1->IsLeaf() && node2->IsLeaf()) {
        if (callback->CollideCallback(nodeId1, nodeId2))
          return true;
      }
      else if (node1->IsLeaf()) {
        stack.emplace(nodeId1, node2->child1);
        stack.emplace(nodeId1, node2->child2);
      }
      else if (node2->IsLeaf()) {
        stack.emplace(node1->child1, nodeId2);
        stack.emplace(node1->child2, nodeId2);
      }
      else {
        stack.emplace(node1->child1, node2->child1);
        stack.emplace(node1->child1, node2->child2);
        stack.emplace(node1->child2, node2->child1);
        stack.emplace(node1->child2, node2->child2);
      }
    }
  }
  return false;
}

template <typename T, typename T2>
bool DistanceRecurse(T * callback, int nodeId, const b2DynamicTree * tree, const T2 & aabb, b2Scalar & dist)
{
  const b2TreeNode * node = tree->GetNode(nodeId);
  if (node->IsLeaf())
    return !callback->DistanceCallback(nodeId, dist);  // stop immediately if intersects
  else {
    b2Scalar d1 = tree->GetFatAABB(node->child1).exteriorDistance(aabb);
    b2Scalar d2 = tree->GetFatAABB(node->child2).exteriorDistance(aabb);
    if (d1 < d2) {
      if (d1 < dist) {
        if (DistanceRecurse(callback, node->child1, tree, aabb, dist))
          return true;
      }
      if (d2 < dist) {
        if (DistanceRecurse(callback, node->child2, tree, aabb, dist))
          return true;
      }
    }
    else {
      if (d2 < dist) {
        if (DistanceRecurse(callback, node->child2, tree, aabb, dist))
          return true;
      }
      if (d1 < dist) {
        if (DistanceRecurse(callback, node->child1, tree, aabb, dist))
          return true;
      }
    }
  }
  return false;
}

template <typename T>
bool b2DynamicTree::Distance(T * callback, const b2AABB & aabb, b2Scalar & dist) const
{
  DistanceRecurse(callback, m_root, this, aabb, dist);
  return dist > b2Scalar(0.0);
}

template <typename T>
bool b2DynamicTree::Distance(T * callback, const b2Vec2 & point, b2Scalar & dist) const
{
  DistanceRecurse(callback, m_root, this, point, dist);
  return dist > b2Scalar(0.0);
}

template <typename T>
bool DistanceRecurse(T * callback, int nodeId1, const b2DynamicTree * tree1, int nodeId2, const b2DynamicTree * tree2, b2Scalar & dist)
{
  const b2TreeNode * node1 = tree1->GetNode(nodeId1);
  const b2TreeNode * node2 = tree2->GetNode(nodeId2);
  if (node1->IsLeaf() && node2->IsLeaf())
    return !callback->DistanceCallback(nodeId1, nodeId2, dist);
  else if (node2->IsLeaf() || (!node1->IsLeaf() && (node1->aabb.volume() > node2->aabb.volume()))) {
    b2Scalar d1 = node2->aabb.exteriorDistance(tree1->GetFatAABB(node1->child1));
    b2Scalar d2 = node2->aabb.exteriorDistance(tree1->GetFatAABB(node1->child2));
    if (d1 < d2) {
      if (d1 < dist) {
        if (DistanceRecurse(callback, node1->child1, tree1, nodeId2, tree2, dist))
          return true;
      }
      if (d2 < dist) {
        if (DistanceRecurse(callback, node1->child2, tree1, nodeId2, tree2, dist))
          return true;
      }
    }
    else {
      if (d2 < dist) {
        if (DistanceRecurse(callback, node1->child2, tree1, nodeId2, tree2, dist))
          return true;
      }
      if (d1 < dist) {
        if (DistanceRecurse(callback, node1->child1, tree1, nodeId2, tree2, dist))
          return true;
      }
    }
  }
  else {
    b2Scalar d1 = node1->aabb.exteriorDistance(tree2->GetFatAABB(node2->child1));
    b2Scalar d2 = node1->aabb.exteriorDistance(tree2->GetFatAABB(node2->child2));
    if (d1 < d2) {
      if (d1 < dist) {
        if (DistanceRecurse(callback, nodeId1, tree1, node2->child1, tree2, dist))
          return true;
      }
      if (d2 < dist) {
        if (DistanceRecurse(callback, nodeId1, tree1, node2->child2, tree2, dist))
          return true;
      }
    }
    else {
      if (d2 < dist) {
        if (DistanceRecurse(callback, nodeId1, tree1, node2->child2, tree2, dist))
          return true;
      }
      if (d1 < dist) {
        if (DistanceRecurse(callback, nodeId1, tree1, node2->child1, tree2, dist))
          return true;
      }
    }
  }
  return false;
}

template <typename T>
bool b2DynamicTree::Distance(T * callback, const b2DynamicTree * tree, b2Scalar & dist) const
{
  DistanceRecurse(callback, m_root, this, tree->GetRoot(), tree, dist);
  return dist > b2Scalar(0.0);
}

template <typename T>
bool SelfDistanceRecurse(T * callback, int nodeId, const b2DynamicTree * tree, b2Scalar & dist)
{
  const b2TreeNode * node = tree->GetNode(nodeId);
  if (node->IsLeaf())
    return false;
  if (SelfDistanceRecurse(callback, node->child1, tree, dist))
    return true;
  if (SelfDistanceRecurse(callback, node->child2, tree, dist))
    return true;
  if (DistanceRecurse(callback, node->child1, tree, node->child2, tree, dist))
    return true;
  return false;
}

template <typename T>
bool b2DynamicTree::SelfDistance(T * callback, b2Scalar & dist) const
{
  SelfDistanceRecurse(callback, m_root, this, dist);
  return dist > b2Scalar(0.0);
}

#endif
