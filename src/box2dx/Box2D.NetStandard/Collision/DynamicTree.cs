/*
  Box2D.NetStandard Copyright © 2020 Ben Ukhanov & Hugh Phoenix-Hulme https://github.com/benzuk/box2d-netstandard
  Box2DX Copyright (c) 2009 Ihar Kalasouski http://code.google.com/p/box2dx
  
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
*/

using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;

namespace Box2D.NetStandard.Collision
{
  internal class DynamicTree
  {
    private int m_root;

    private TreeNode[] m_nodes;
    private int m_nodeCount;
    private int m_nodeCapacity;

    private int m_freeList;

    public DynamicTree()
    {
      m_root = -1;

      m_nodeCapacity = 16;
      m_nodeCount = 0;
      m_nodes = new TreeNode[m_nodeCapacity];

      // Build a linked list for the free list.
      for (int i = 0; i < m_nodeCapacity; ++i) {
        m_nodes[i] = new TreeNode();
        m_nodes[i].pn.next = i + 1;
        m_nodes[i].height = -1;
      }

      m_nodes[m_nodeCapacity - 1].pn.next = -1;
      m_nodes[m_nodeCapacity - 1].height = -1;
      m_freeList = 0;
    }

    public int AllocateNode()
    {
      // Expand the node pool as needed.
      if (m_freeList == -1) {
        //Debug.Assert(m_nodeCount == m_nodeCapacity);

        // The free list is empty. Rebuild a bigger pool.
        TreeNode[] oldNodes = m_nodes;
        m_nodeCapacity *= 2;
        m_nodes = new TreeNode[m_nodeCapacity];
        Array.Copy(oldNodes, m_nodes, m_nodeCount);

        // Build a linked list for the free list. The parent
        // pointer becomes the "next" pointer.
        for (int i = m_nodeCount; i < m_nodeCapacity; ++i) {
          m_nodes[i] = new TreeNode();
          m_nodes[i].pn.next = i + 1;
          m_nodes[i].height = -1;
        }

        m_nodes[m_nodeCapacity - 1].pn.next = -1;
        m_nodes[m_nodeCapacity - 1].height = -1;
        m_freeList = m_nodeCount;
      }

      // Peel a node off the free list.
      int nodeId = m_freeList;
      m_freeList = m_nodes[nodeId].pn.next;
      m_nodes[nodeId].pn.parent = -1;
      m_nodes[nodeId].child1 = -1;
      m_nodes[nodeId].child2 = -1;
      m_nodes[nodeId].height = 0;
      m_nodes[nodeId].userData = null;
      m_nodes[nodeId].moved = false;
      ++m_nodeCount;
      return nodeId;
    }

    public void FreeNode(int nodeId)
    {
      //Debug.Assert(0 <= nodeId && nodeId < m_nodeCapacity);
      //Debug.Assert(0 < m_nodeCount);
      m_nodes[nodeId].pn.next = m_freeList;
      m_nodes[nodeId].height = -1;
      m_freeList = nodeId;
      --m_nodeCount;
    }


    public int CreateProxy(in AABB aabb, object userData)
    {
      int proxyId = AllocateNode();

      // Fatten the aabb.
      Vector2 r = new Vector2(Settings.AABBExtension, Settings.AABBExtension);
      m_nodes[proxyId].aabb.lowerBound = aabb.lowerBound - r;
      m_nodes[proxyId].aabb.upperBound = aabb.upperBound + r;
      m_nodes[proxyId].userData = userData;
      m_nodes[proxyId].height = 0;
      m_nodes[proxyId].moved = true;

      InsertLeaf(proxyId);

      return proxyId;
    }

    public void DestroyProxy(int proxyId)
    {
      //Debug.Assert(0 <= proxyId && proxyId < m_nodeCapacity);
      //Debug.Assert(m_nodes[proxyId].IsLeaf());

      RemoveLeaf(proxyId);
      FreeNode(proxyId);
    }

    public bool MoveProxy(Int32 proxyId, in AABB aabb, in Vector2 displacement)
    {
      //Debug.Assert(0 <= proxyId && proxyId < m_nodeCapacity);

      //Debug.Assert(m_nodes[proxyId].IsLeaf());

      // Extend AABB
      AABB fatAABB;
      Vector2 r = new Vector2(Settings.AABBExtension, Settings.AABBExtension);
      fatAABB.lowerBound = aabb.lowerBound - r;
      fatAABB.upperBound = aabb.upperBound + r;

      // Predict AABB movement
      Vector2 d = Settings.AABBMultiplier * displacement;

      if (d.X < 0.0f) {
        fatAABB.lowerBound.X += d.X;
      }
      else {
        fatAABB.upperBound.X += d.X;
      }

      if (d.Y < 0.0f) {
        fatAABB.lowerBound.Y += d.Y;
      }
      else {
        fatAABB.upperBound.Y += d.Y;
      }

      AABB treeAABB = m_nodes[proxyId].aabb;
      if (treeAABB.Contains(aabb)) {
        // The tree AABB still contains the object, but it might be too large.
        // Perhaps the object was moving fast but has since gone to sleep.
        // The huge AABB is larger than the new fat AABB.
        AABB hugeAABB;
        hugeAABB.lowerBound = fatAABB.lowerBound - 4.0f * r;
        hugeAABB.upperBound = fatAABB.upperBound + 4.0f * r;

        if (hugeAABB.Contains(treeAABB)) {
          // The tree AABB contains the object AABB and the tree AABB is
          // not too large. No tree update needed.
          return false;
        }

        // Otherwise the tree AABB is huge and needs to be shrunk
      }

      RemoveLeaf(proxyId);

      m_nodes[proxyId].aabb = fatAABB;

      InsertLeaf(proxyId);

      m_nodes[proxyId].moved = true;

      return true;
    }

    public void InsertLeaf(Int32 leaf)
    {
      if (m_root == -1) {
        m_root = leaf;
        m_nodes[m_root].pn.parent = -1;
        return;
      }

      // Find the best sibling for this node
      AABB leafAABB = m_nodes[leaf].aabb;
      Int32 index = m_root;
      while (m_nodes[index].IsLeaf() == false) {
        Int32 child1 = m_nodes[index].child1;
        Int32 child2 = m_nodes[index].child2;

        float area = m_nodes[index].aabb.GetPerimeter();

        AABB combinedAABB = new AABB();
        combinedAABB.Combine(m_nodes[index].aabb, leafAABB);
        float combinedArea = combinedAABB.GetPerimeter();

        // Cost of creating a new parent for this node and the new leaf
        float cost = 2.0f * combinedArea;

        // Minimum cost of pushing the leaf further down the tree
        float inheritanceCost = 2.0f * (combinedArea - area);

        // Cost of descending into child1
        float cost1;
        if (m_nodes[child1].IsLeaf()) {
          AABB aabb = new AABB();
          aabb.Combine(leafAABB, m_nodes[child1].aabb);
          cost1 = aabb.GetPerimeter() + inheritanceCost;
        }
        else {
          AABB aabb = new AABB();
          aabb.Combine(leafAABB, m_nodes[child1].aabb);
          float oldArea = m_nodes[child1].aabb.GetPerimeter();
          float newArea = aabb.GetPerimeter();
          cost1 = (newArea - oldArea) + inheritanceCost;
        }

        // Cost of descending into child2
        float cost2;
        if (m_nodes[child2].IsLeaf()) {
          AABB aabb = new AABB();
          aabb.Combine(leafAABB, m_nodes[child2].aabb);
          cost2 = aabb.GetPerimeter() + inheritanceCost;
        }
        else {
          AABB aabb = new AABB();
          aabb.Combine(leafAABB, m_nodes[child2].aabb);
          float oldArea = m_nodes[child2].aabb.GetPerimeter();
          float newArea = aabb.GetPerimeter();
          cost2 = newArea - oldArea + inheritanceCost;
        }

        // Descend according to the minimum cost.
        if (cost < cost1 && cost < cost2) {
          break;
        }

        // Descend
        if (cost1 < cost2) {
          index = child1;
        }
        else {
          index = child2;
        }
      }

      Int32 sibling = index;

      // Create a new parent.
      Int32 oldParent = m_nodes[sibling].pn.parent;
      Int32 newParent = AllocateNode();
      m_nodes[newParent].pn.parent = oldParent;
      m_nodes[newParent].userData = null;
      m_nodes[newParent].aabb.Combine(leafAABB, m_nodes[sibling].aabb);
      m_nodes[newParent].height = m_nodes[sibling].height + 1;

      if (oldParent != -1) {
        // The sibling was not the root.
        if (m_nodes[oldParent].child1 == sibling) {
          m_nodes[oldParent].child1 = newParent;
        }
        else {
          m_nodes[oldParent].child2 = newParent;
        }

        m_nodes[newParent].child1 = sibling;
        m_nodes[newParent].child2 = leaf;
        m_nodes[sibling].pn.parent = newParent;
        m_nodes[leaf].pn.parent = newParent;
      }
      else {
        // The sibling was the root.
        m_nodes[newParent].child1 = sibling;
        m_nodes[newParent].child2 = leaf;
        m_nodes[sibling].pn.parent = newParent;
        m_nodes[leaf].pn.parent = newParent;
        m_root = newParent;
      }

      // Walk back up the tree fixing heights and AABBs
      index = m_nodes[leaf].pn.parent;
      while (index != -1) {
        index = Balance(index);

        Int32 child1 = m_nodes[index].child1;
        Int32 child2 = m_nodes[index].child2;

        //Debug.Assert(child1 != -1);
        //Debug.Assert(child2 != -1);

        m_nodes[index].height = 1 + System.Math.Max(m_nodes[child1].height, m_nodes[child2].height);
        m_nodes[index].aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);

        index = m_nodes[index].pn.parent;

        //VisualiseTree();
      }

      //Validate();
    }

    // private void VisualiseTree() {
    //   Console.WriteLine("---- TREE ----");
    //   
    //   for (int i = 0; i < m_nodeCount; i++) {
    //     TreeNode node = m_nodes[i];
    //     Console.WriteLine($"Node: {i} Parent: {node.pn.parent} Child1: {node.child1} Child2: {node.child2}");
    //   }
    // }

    public void RemoveLeaf(Int32 leaf)
    {
      if (leaf == m_root) {
        m_root = -1;
        return;
      }

      Int32 parent = m_nodes[leaf].pn.parent;
      Int32 grandParent = m_nodes[parent].pn.parent;
      Int32 sibling;
      if (m_nodes[parent].child1 == leaf) {
        sibling = m_nodes[parent].child2;
      }
      else {
        sibling = m_nodes[parent].child1;
      }

      if (grandParent != -1) {
        // Destroy parent and connect sibling to grandParent.
        if (m_nodes[grandParent].child1 == parent) {
          m_nodes[grandParent].child1 = sibling;
        }
        else {
          m_nodes[grandParent].child2 = sibling;
        }

        m_nodes[sibling].pn.parent = grandParent;
        FreeNode(parent);

        // Adjust ancestor bounds.
        Int32 index = grandParent;
        while (index != -1) {
          index = Balance(index);

          Int32 child1 = m_nodes[index].child1;
          Int32 child2 = m_nodes[index].child2;

          m_nodes[index].aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);
          m_nodes[index].height = 1 + System.Math.Max(m_nodes[child1].height, m_nodes[child2].height);

          index = m_nodes[index].pn.parent;
        }
      }
      else {
        m_root = sibling;
        m_nodes[sibling].pn.parent = -1;
        FreeNode(parent);
      }

      //Validate();
    }

    public Int32 Balance(Int32 iA)
    {
      //Debug.Assert(iA != -1);

      TreeNode A = m_nodes[iA];
      if (A.IsLeaf() || A.height < 2) {
        return iA;
      }

      Int32 iB = A.child1;
      Int32 iC = A.child2;
      //Debug.Assert(0 <= iB && iB < m_nodeCapacity);
      //Debug.Assert(0 <= iC && iC < m_nodeCapacity);

      TreeNode B = m_nodes[iB];
      TreeNode C = m_nodes[iC];

      Int32 balance = C.height - B.height;

      // Rotate C up
      if (balance > 1) {
        Int32 iF = C.child1;
        Int32 iG = C.child2;
        TreeNode F = m_nodes[iF];
        TreeNode G = m_nodes[iG];
        //Debug.Assert(0 <= iF && iF < m_nodeCapacity);
        //Debug.Assert(0 <= iG && iG < m_nodeCapacity);

        // Swap A and C
        C.child1 = iA;
        C.pn.parent = A.pn.parent;
        A.pn.parent = iC;

        // A's old parent should point to C
        if (C.pn.parent != -1) {
          if (m_nodes[C.pn.parent].child1 == iA) {
            m_nodes[C.pn.parent].child1 = iC;
          }
          else {
            //Debug.Assert(m_nodes[C.pn.parent].child2 == iA);
            m_nodes[C.pn.parent].child2 = iC;
          }
        }
        else {
          m_root = iC;
        }

        // Rotate
        if (F.height > G.height) {
          C.child2 = iF;
          A.child2 = iG;
          G.pn.parent = iA;
          A.aabb.Combine(B.aabb, G.aabb);
          C.aabb.Combine(A.aabb, F.aabb);

          A.height = 1 + System.Math.Max(B.height, G.height);
          C.height = 1 + System.Math.Max(A.height, F.height);
        }
        else {
          C.child2 = iG;
          A.child2 = iF;
          F.pn.parent = iA;
          A.aabb.Combine(B.aabb, F.aabb);
          C.aabb.Combine(A.aabb, G.aabb);

          A.height = 1 + System.Math.Max(B.height, F.height);
          C.height = 1 + System.Math.Max(A.height, G.height);
        }

        return iC;
      }

      // Rotate B up
      if (balance < -1) {
        Int32 iD = B.child1;
        Int32 iE = B.child2;
        TreeNode D = m_nodes[iD];
        TreeNode E = m_nodes[iE];
        //Debug.Assert(0 <= iD && iD < m_nodeCapacity);
        //Debug.Assert(0 <= iE && iE < m_nodeCapacity);

        // Swap A and B
        B.child1 = iA;
        B.pn.parent = A.pn.parent;
        A.pn.parent = iB;

        // A's old parent should point to B
        if (B.pn.parent != -1) {
          if (m_nodes[B.pn.parent].child1 == iA) {
            m_nodes[B.pn.parent].child1 = iB;
          }
          else {
            //Debug.Assert(m_nodes[B.pn.parent].child2 == iA);
            m_nodes[B.pn.parent].child2 = iB;
          }
        }
        else {
          m_root = iB;
        }

        // Rotate
        if (D.height > E.height) {
          B.child2 = iD;
          A.child1 = iE;
          E.pn.parent = iA;
          A.aabb.Combine(C.aabb, E.aabb);
          B.aabb.Combine(A.aabb, D.aabb);

          A.height = 1 + System.Math.Max(C.height, E.height);
          B.height = 1 + System.Math.Max(A.height, D.height);
        }
        else {
          B.child2 = iE;
          A.child1 = iD;
          D.pn.parent = iA;
          A.aabb.Combine(C.aabb, D.aabb);
          B.aabb.Combine(A.aabb, E.aabb);

          A.height = 1 + System.Math.Max(C.height, D.height);
          B.height = 1 + System.Math.Max(A.height, E.height);
        }

        return iB;
      }

      return iA;
    }

    public Int32 GetHeight()
    {
      if (m_root == -1) {
        return 0;
      }

      return m_nodes[m_root].height;
    }

    public float GetAreaRatio()
    {
      if (m_root == -1) {
        return 0.0f;
      }

      TreeNode root = m_nodes[m_root];
      float rootArea = root.aabb.GetPerimeter();

      float totalArea = 0.0f;
      for (Int32 i = 0; i < m_nodeCapacity; ++i) {
        TreeNode node = m_nodes[i];
        if (node.height < 0) {
          // Free node in pool
          continue;
        }

        totalArea += node.aabb.GetPerimeter();
      }

      return totalArea / rootArea;
    }

    public int ComputeHeight(Int32 nodeId)
    {
      //Debug.Assert(0 <= nodeId && nodeId < m_nodeCapacity);
      TreeNode node = m_nodes[nodeId];

      if (node.IsLeaf()) {
        return 0;
      }

      Int32 height1 = ComputeHeight(node.child1);
      Int32 height2 = ComputeHeight(node.child2);
      return 1 + System.Math.Max(height1, height2);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int ComputeHeight() => ComputeHeight(m_root);

    public void ValidateStructure(Int32 index)
    {
      if (index == -1) {
        return;
      }

      if (index == m_root) {
        //Debug.Assert(m_nodes[index].pn.parent == -1);
      }

      TreeNode node = m_nodes[index];

      Int32 child1 = node.child1;
      Int32 child2 = node.child2;

      if (node.IsLeaf()) {
        //Debug.Assert(child1      == -1);
        //Debug.Assert(child2      == -1);
        //Debug.Assert(node.height == 0);
        return;
      }

      //Debug.Assert(0 <= child1 && child1 < m_nodeCapacity);
      //Debug.Assert(0 <= child2 && child2 < m_nodeCapacity);
      //Debug.Assert(m_nodes[child1].pn.parent == index);
      //Debug.Assert(m_nodes[child2].pn.parent == index);

      ValidateStructure(child1);
      ValidateStructure(child2);
    }

    public void ValidateMetrics(Int32 index)
    {
      if (index == -1) {
        return;
      }

      TreeNode node = m_nodes[index];

      Int32 child1 = node.child1;
      Int32 child2 = node.child2;

      if (node.IsLeaf()) {
        //Debug.Assert(child1      == -1);
        //Debug.Assert(child2      == -1);
        //Debug.Assert(node.height == 0);
        return;
      }

      //Debug.Assert(0 <= child1 && child1 < m_nodeCapacity);
      //Debug.Assert(0 <= child2 && child2 < m_nodeCapacity);

      Int32 height1 = m_nodes[child1].height;
      Int32 height2 = m_nodes[child2].height;
      Int32 height;
      height = 1 + System.Math.Max(height1, height2);
      //Debug.Assert(node.height == height);

      AABB aabb = new AABB();
      aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);

      //Debug.Assert(aabb.lowerBound == node.aabb.lowerBound);
      //Debug.Assert(aabb.upperBound == node.aabb.upperBound);

      ValidateMetrics(child1);
      ValidateMetrics(child2);
    }

    public int GetMaxBalance()
    {
      Int32 maxBalance = 0;
      for (Int32 i = 0; i < m_nodeCapacity; ++i) {
        TreeNode node = m_nodes[i];
        if (node.height <= 1) {
          continue;
        }

        //Debug.Assert(node.IsLeaf() == false);

        Int32 child1 = node.child1;
        Int32 child2 = node.child2;
        Int32 balance = System.Math.Abs(m_nodes[child2].height - m_nodes[child1].height);
        maxBalance = System.Math.Max(maxBalance, balance);
      }

      return maxBalance;
    }

    public void RebuildBottomUp()
    {
      Int32[] nodes = new int[m_nodeCount];
      Int32 count = 0;

      // Build array of leaves. Free the rest.
      for (Int32 i = 0; i < m_nodeCapacity; ++i) {
        if (m_nodes[i].height < 0) {
          // free node in pool
          continue;
        }

        if (m_nodes[i].IsLeaf()) {
          m_nodes[i].pn.parent = -1;
          nodes[count] = i;
          ++count;
        }
        else {
          FreeNode(i);
        }
      }

      while (count > 1) {
        float minCost = float.MaxValue;
        Int32 iMin = -1, jMin = -1;
        for (Int32 i = 0; i < count; ++i) {
          AABB aabbi = m_nodes[nodes[i]].aabb;

          for (Int32 j = i + 1; j < count; ++j) {
            AABB aabbj = m_nodes[nodes[j]].aabb;
            AABB b = new AABB();
            b.Combine(in aabbi, in aabbj);
            float cost = b.GetPerimeter();
            if (cost < minCost) {
              iMin = i;
              jMin = j;
              minCost = cost;
            }
          }
        }

        Int32 index1 = nodes[iMin];
        Int32 index2 = nodes[jMin];
        TreeNode child1 = m_nodes[index1];
        TreeNode child2 = m_nodes[index2];

        Int32 parentIndex = AllocateNode();
        TreeNode parent = m_nodes[parentIndex];
        parent.child1 = index1;
        parent.child2 = index2;
        parent.height = 1 + System.Math.Max(child1.height, child2.height);
        parent.aabb.Combine(child1.aabb, child2.aabb);
        parent.pn.parent = -1;

        child1.pn.parent = parentIndex;
        child2.pn.parent = parentIndex;

        nodes[jMin] = nodes[count - 1];
        nodes[iMin] = parentIndex;
        --count;
      }

      m_root = nodes[0];

      //Validate();
    }

    public void ShiftOrigin(in Vector2 newOrigin)
    {
      // Build array of leaves. Free the rest.
      for (Int32 i = 0; i < m_nodeCapacity; ++i) {
        m_nodes[i].aabb.lowerBound -= newOrigin;
        m_nodes[i].aabb.upperBound -= newOrigin;
      }
    }

    public object GetUserData(int proxyId) => m_nodes[proxyId].userData;
    public bool WasMoved(int proxyId) => m_nodes[proxyId].moved;
    public void ClearMoved(int proxyId) => m_nodes[proxyId].moved = false;

    public AABB GetFatAABB(int proxyId) => m_nodes[proxyId].aabb;

    public void Query(Func<int, bool> queryCallback, in AABB aabb)
    {
      Stack<int> stack = new Stack<int>(256);
      stack.Push(m_root);

      while (stack.Count > 0) {
        int nodeId = stack.Pop();
        if (nodeId == -1) {
          continue;
        }

        TreeNode node = m_nodes[nodeId];

        if (Collision.TestOverlap(node.aabb, aabb)) {
          if (node.IsLeaf()) {
            bool proceed = queryCallback(nodeId);
            if (proceed == false) {
              return;
            }
          }
          else {
            stack.Push(node.child1);
            stack.Push(node.child2);
          }
        }
      }
    }

    public void RayCast(Func<RayCastInput, int, float> RayCastCallback, in RayCastInput input)
    {
      Vector2 p1 = input.p1;
      Vector2 p2 = input.p2;
      Vector2 r = p2 - p1;
      //Debug.Assert(r.LengthSquared() > 0.0f);
      r = Vector2.Normalize(r);

      // v is perpendicular to the segment.
      Vector2 v = Vectex.Cross(1.0f, r);
      Vector2 abs_v = Vector2.Abs(v);

      // Separating axis for segment (Gino, p80).
      // |dot(v, p1 - c)| > dot(|v|, h)

      float maxFraction = input.maxFraction;

      // Build a bounding box for the segment.
      AABB segmentAABB;
      {
        Vector2 t = p1 + maxFraction * (p2 - p1);
        segmentAABB.lowerBound = Vector2.Min(p1, t);
        segmentAABB.upperBound = Vector2.Max(p1, t);
      }

      Stack<int> stack = new Stack<int>(256);
      stack.Push(m_root);

      while (stack.Count > 0) {
        int nodeId = stack.Pop();
        if (nodeId == -1) {
          continue;
        }

        TreeNode node = m_nodes[nodeId];

        if (Collision.TestOverlap(node.aabb, segmentAABB) == false) {
          continue;
        }

        // Separating axis for segment (Gino, p80).
        // |dot(v, p1 - c)| > dot(|v|, h)
        Vector2 c = node.aabb.GetCenter();
        Vector2 h = node.aabb.GetExtents();
        float separation = MathF.Abs(Vector2.Dot(v, p1 - c)) - Vector2.Dot(abs_v, h);
        if (separation > 0.0f) {
          continue;
        }

        if (node.IsLeaf()) {
          RayCastInput subInput;
          subInput.p1 = input.p1;
          subInput.p2 = input.p2;
          subInput.maxFraction = maxFraction;

          float value = RayCastCallback(subInput, nodeId);

          if (value == 0.0f) {
            // The client has terminated the ray cast.
            return;
          }

          if (value > 0.0f) {
            // Update segment bounding box.
            maxFraction = value;
            Vector2 t = p1 + maxFraction * (p2 - p1);
            segmentAABB.lowerBound = Vector2.Min(p1, t);
            segmentAABB.upperBound = Vector2.Max(p1, t);
          }
        }
        else {
          stack.Push(node.child1);
          stack.Push(node.child2);
        }
      }
    }
  }
}