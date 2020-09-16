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
using Math = System.Math;

namespace Box2D.NetStandard.Collision
{
	internal class DynamicTree
	{
		private static readonly Vector2 r = new Vector2(Settings.AABBExtension);
		private int m_freeList;
		private int m_nodeCapacity;
		private int m_nodeCount;
		private TreeNode[] m_nodes;
		private int m_root;

		public DynamicTree()
		{
			m_root = -1;

			m_nodeCapacity = 2048;
			m_nodeCount = 0;
			m_nodes = new TreeNode[m_nodeCapacity];

			// Build a linked list for the free list.
			for (var i = 0; i < m_nodeCapacity; ++i)
			{
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
			if (m_freeList == -1)
			{
				//Debug.Assert(m_nodeCount == m_nodeCapacity);

				// The free list is empty. Rebuild a bigger pool.
				m_nodeCapacity *= 2;
				TreeNode[] oldNodes = m_nodes;
				m_nodes = new TreeNode[m_nodeCapacity];
				Array.Copy(oldNodes, m_nodes, m_nodeCount);

				// Build a linked list for the free list. The parent
				// pointer becomes the "next" pointer.
				for (int i = m_nodeCount; i < m_nodeCapacity; ++i)
				{
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

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void FreeNode(in int nodeId)
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

			m_nodes[proxyId].aabb.lowerBound = aabb.lowerBound - r;
			m_nodes[proxyId].aabb.upperBound = aabb.upperBound + r;
			m_nodes[proxyId].userData = userData;
			m_nodes[proxyId].height = 0;
			m_nodes[proxyId].moved = true;

			InsertLeaf(proxyId);

			return proxyId;
		}

		public void DestroyProxy(in int proxyId)
		{
			//Debug.Assert(0 <= proxyId && proxyId < m_nodeCapacity);
			//Debug.Assert(m_nodes[proxyId].IsLeaf());

			RemoveLeaf(proxyId);
			FreeNode(proxyId);
		}

		public bool MoveProxy(in int proxyId, in AABB aabb, in Vector2 displacement)
		{
			//Debug.Assert(0 <= proxyId && proxyId < m_nodeCapacity);

			//Debug.Assert(m_nodes[proxyId].IsLeaf());

			// Extend AABB
			var fatAABB = new AABB(aabb.lowerBound - r, aabb.UpperBound + r);

			// Predict AABB movement
			Vector2 d = Settings.AABBMultiplier * displacement;

			if (d.X < 0.0f)
			{
				fatAABB.lowerBound.X += d.X;
			}
			else
			{
				fatAABB.upperBound.X += d.X;
			}

			if (d.Y < 0.0f)
			{
				fatAABB.lowerBound.Y += d.Y;
			}
			else
			{
				fatAABB.upperBound.Y += d.Y;
			}

			AABB treeAABB = m_nodes[proxyId].aabb;
			if (treeAABB.Contains(aabb))
			{
				// The tree AABB still contains the object, but it might be too large.
				// Perhaps the object was moving fast but has since gone to sleep.
				// The huge AABB is larger than the new fat AABB.
				var hugeAABB = new AABB(fatAABB.lowerBound - 4 * r, fatAABB.upperBound + 4 * r);

				if (hugeAABB.Contains(treeAABB))
					// The tree AABB contains the object AABB and the tree AABB is
					// not too large. No tree update needed.
				{
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

		public void InsertLeaf(in int leaf)
		{
			if (m_root == -1)
			{
				m_root = leaf;
				m_nodes[m_root].pn.parent = -1;
				return;
			}

			// Find the best sibling for this node
			AABB leafAABB = m_nodes[leaf].aabb;
			int index = m_root;
			while (!m_nodes[index].IsLeaf())
			{
				int child1 = m_nodes[index].child1;
				int child2 = m_nodes[index].child2;

				float area = m_nodes[index].aabb.GetPerimeter();

				var combinedAABB = new AABB();
				combinedAABB.Combine(m_nodes[index].aabb, leafAABB);
				float combinedArea = combinedAABB.GetPerimeter();

				// Cost of creating a new parent for this node and the new leaf
				float cost = 2.0f * combinedArea;

				// Minimum cost of pushing the leaf further down the tree
				float inheritanceCost = 2.0f * (combinedArea - area);

				float cost1 = GetChildDescentCost(leafAABB, child1) + inheritanceCost;
				float cost2 = GetChildDescentCost(leafAABB, child2) + inheritanceCost;

				if (cost < cost1 && cost < cost2)
				{
					break;
				}

				index = cost1 < cost2 ? child1 : child2;
			}

			int sibling = index;

			// Create a new parent.
			int oldParent = m_nodes[sibling].pn.parent;
			int newParent = AllocateNode();
			m_nodes[newParent].pn.parent = oldParent;
			m_nodes[newParent].userData = null;
			m_nodes[newParent].aabb.Combine(leafAABB, m_nodes[sibling].aabb);
			m_nodes[newParent].height = m_nodes[sibling].height + 1;
			m_nodes[newParent].child1 = sibling;
			m_nodes[newParent].child2 = leaf;
			m_nodes[sibling].pn.parent = newParent;
			m_nodes[leaf].pn.parent = newParent;

			if (oldParent != -1)
			{
				// The sibling was not the root.
				if (m_nodes[oldParent].child1 == sibling)
				{
					m_nodes[oldParent].child1 = newParent;
				}
				else
				{
					m_nodes[oldParent].child2 = newParent;
				}
			}
			else
			{
				// The sibling was the root.
				m_root = newParent;
			}

			// Walk back up the tree fixing heights and AABBs
			AdjustBounds(m_nodes[leaf].pn.parent);

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

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private float GetChildDescentCost(in AABB leafAABB, in int child)
		{
			float cost;
			AABB aabb = default;
			aabb.Combine(leafAABB, m_nodes[child].aabb);
			if (m_nodes[child].IsLeaf())
			{
				cost = aabb.GetPerimeter();
			}
			else
			{
				float oldArea = m_nodes[child].aabb.GetPerimeter();
				float newArea = aabb.GetPerimeter();
				cost = newArea - oldArea;
			}

			return cost;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private void AdjustBounds(in int idx)
		{
			int index = idx;
			while (index != -1)
			{
				index = Balance(index);

				int child1 = m_nodes[index].child1;
				int child2 = m_nodes[index].child2;

				m_nodes[index].height = 1 + Math.Max(m_nodes[child1].height, m_nodes[child2].height);
				m_nodes[index].aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);

				index = m_nodes[index].pn.parent;
			}
		}

		public void RemoveLeaf(in int leaf)
		{
			if (leaf == m_root)
			{
				m_root = -1;
				return;
			}

			int parent = m_nodes[leaf].pn.parent;
			int grandParent = m_nodes[parent].pn.parent;
			int sibling = m_nodes[parent].child1 == leaf ? m_nodes[parent].child2 : m_nodes[parent].child1;

			if (grandParent == -1)
			{
				m_root = sibling;
				m_nodes[sibling].pn.parent = -1;
				FreeNode(parent);
				return;
			}

			// Destroy parent and connect sibling to grandParent.
			if (m_nodes[grandParent].child1 == parent)
			{
				m_nodes[grandParent].child1 = sibling;
			}
			else
			{
				m_nodes[grandParent].child2 = sibling;
			}

			m_nodes[sibling].pn.parent = grandParent;
			FreeNode(parent);

			AdjustBounds(grandParent);

			//Validate();
		}

		public int Balance(in int iA)
		{
			//Debug.Assert(iA != -1);

			TreeNode A = m_nodes[iA];
			if (A.IsLeaf() || A.height < 2)
			{
				return iA;
			}

			int iB = A.child1;
			int iC = A.child2;
			//Debug.Assert(0 <= iB && iB < m_nodeCapacity);
			//Debug.Assert(0 <= iC && iC < m_nodeCapacity);

			TreeNode B = m_nodes[iB];
			TreeNode C = m_nodes[iC];

			int balance = C.height - B.height;

			// Rotate C up
			if (balance > 1)
			{
				int iF = C.child1;
				int iG = C.child2;
				TreeNode F = m_nodes[iF];
				TreeNode G = m_nodes[iG];
				//Debug.Assert(0 <= iF && iF < m_nodeCapacity);
				//Debug.Assert(0 <= iG && iG < m_nodeCapacity);

				// Swap A and C
				C.child1 = iA;
				C.pn.parent = A.pn.parent;
				A.pn.parent = iC;

				// A's old parent should point to C
				if (C.pn.parent != -1)
				{
					if (m_nodes[C.pn.parent].child1 == iA)
					{
						m_nodes[C.pn.parent].child1 = iC;
					}
					else
						//Debug.Assert(m_nodes[C.pn.parent].child2 == iA);
					{
						m_nodes[C.pn.parent].child2 = iC;
					}
				}
				else
				{
					m_root = iC;
				}

				// Rotate
				if (F.height > G.height)
				{
					C.child2 = iF;
					A.child2 = iG;
					G.pn.parent = iA;
					A.aabb.Combine(B.aabb, G.aabb);
					C.aabb.Combine(A.aabb, F.aabb);

					A.height = 1 + Math.Max(B.height, G.height);
					C.height = 1 + Math.Max(A.height, F.height);
				}
				else
				{
					C.child2 = iG;
					A.child2 = iF;
					F.pn.parent = iA;
					A.aabb.Combine(B.aabb, F.aabb);
					C.aabb.Combine(A.aabb, G.aabb);

					A.height = 1 + Math.Max(B.height, F.height);
					C.height = 1 + Math.Max(A.height, G.height);
				}

				return iC;
			}

			// Rotate B up
			if (balance < -1)
			{
				int iD = B.child1;
				int iE = B.child2;
				TreeNode D = m_nodes[iD];
				TreeNode E = m_nodes[iE];
				//Debug.Assert(0 <= iD && iD < m_nodeCapacity);
				//Debug.Assert(0 <= iE && iE < m_nodeCapacity);

				// Swap A and B
				B.child1 = iA;
				B.pn.parent = A.pn.parent;
				A.pn.parent = iB;

				// A's old parent should point to B
				if (B.pn.parent != -1)
				{
					if (m_nodes[B.pn.parent].child1 == iA)
					{
						m_nodes[B.pn.parent].child1 = iB;
					}
					else
						//Debug.Assert(m_nodes[B.pn.parent].child2 == iA);
					{
						m_nodes[B.pn.parent].child2 = iB;
					}
				}
				else
				{
					m_root = iB;
				}

				// Rotate
				if (D.height > E.height)
				{
					B.child2 = iD;
					A.child1 = iE;
					E.pn.parent = iA;
					A.aabb.Combine(C.aabb, E.aabb);
					B.aabb.Combine(A.aabb, D.aabb);

					A.height = 1 + Math.Max(C.height, E.height);
					B.height = 1 + Math.Max(A.height, D.height);
				}
				else
				{
					B.child2 = iE;
					A.child1 = iD;
					D.pn.parent = iA;
					A.aabb.Combine(C.aabb, D.aabb);
					B.aabb.Combine(A.aabb, E.aabb);

					A.height = 1 + Math.Max(C.height, D.height);
					B.height = 1 + Math.Max(A.height, E.height);
				}

				return iB;
			}

			return iA;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public int GetHeight() => m_root == -1 ? 0 : m_nodes[m_root].height;

		public float GetAreaRatio()
		{
			if (m_root == -1)
			{
				return 0.0f;
			}

			TreeNode root = m_nodes[m_root];
			float rootArea = root.aabb.GetPerimeter();

			var totalArea = 0.0f;
			for (var i = 0; i < m_nodeCapacity; ++i)
			{
				TreeNode node = m_nodes[i];
				if (node.height < 0)
					// Free node in pool
				{
					continue;
				}

				totalArea += node.aabb.GetPerimeter();
			}

			return totalArea / rootArea;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public int ComputeHeight(int nodeId)
		{
			//Debug.Assert(0 <= nodeId && nodeId < m_nodeCapacity);
			TreeNode node = m_nodes[nodeId];

			if (node.IsLeaf())
			{
				return 0;
			}

			int height1 = ComputeHeight(node.child1);
			int height2 = ComputeHeight(node.child2);
			return 1 + Math.Max(height1, height2);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public int ComputeHeight() => ComputeHeight(m_root);

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void ValidateStructure(int index)
		{
			if (index == -1)
			{
				return;
			}

			// if (index == m_root)
			// {
			//   //Debug.Assert(m_nodes[index].pn.parent == -1);
			// }

			TreeNode node = m_nodes[index];

			int child1 = node.child1;
			int child2 = node.child2;

			if (node.IsLeaf())
				//Debug.Assert(child1      == -1);
				//Debug.Assert(child2      == -1);
				//Debug.Assert(node.height == 0);
			{
				return;
			}

			//Debug.Assert(0 <= child1 && child1 < m_nodeCapacity);
			//Debug.Assert(0 <= child2 && child2 < m_nodeCapacity);
			//Debug.Assert(m_nodes[child1].pn.parent == index);
			//Debug.Assert(m_nodes[child2].pn.parent == index);

			ValidateStructure(child1);
			ValidateStructure(child2);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void ValidateMetrics(int index)
		{
			if (index == -1)
			{
				return;
			}

			TreeNode node = m_nodes[index];

			int child1 = node.child1;
			int child2 = node.child2;

			if (node.IsLeaf())
				//Debug.Assert(child1      == -1);
				//Debug.Assert(child2      == -1);
				//Debug.Assert(node.height == 0);
			{
				return;
			}

			//Debug.Assert(0 <= child1 && child1 < m_nodeCapacity);
			//Debug.Assert(0 <= child2 && child2 < m_nodeCapacity);

			int height1 = m_nodes[child1].height;
			int height2 = m_nodes[child2].height;
			int height;
			height = 1 + Math.Max(height1, height2);
			//Debug.Assert(node.height == height);

			var aabb = new AABB();
			aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);

			//Debug.Assert(aabb.lowerBound == node.aabb.lowerBound);
			//Debug.Assert(aabb.upperBound == node.aabb.upperBound);

			ValidateMetrics(child1);
			ValidateMetrics(child2);
		}

		public int GetMaxBalance()
		{
			var maxBalance = 0;
			for (var i = 0; i < m_nodeCapacity; ++i)
			{
				TreeNode node = m_nodes[i];
				if (node.height <= 1)
				{
					continue;
				}

				//Debug.Assert(node.IsLeaf() == false);

				int child1 = node.child1;
				int child2 = node.child2;
				int balance = Math.Abs(m_nodes[child2].height - m_nodes[child1].height);
				maxBalance = Math.Max(maxBalance, balance);
			}

			return maxBalance;
		}

		public void RebuildBottomUp()
		{
			var nodes = new int[m_nodeCount];
			var count = 0;

			// Build array of leaves. Free the rest.
			for (var i = 0; i < m_nodeCapacity; ++i)
			{
				if (m_nodes[i].height < 0)
					// free node in pool
				{
					continue;
				}

				if (m_nodes[i].IsLeaf())
				{
					m_nodes[i].pn.parent = -1;
					nodes[count] = i;
					++count;
				}
				else
				{
					FreeNode(i);
				}
			}

			while (count > 1)
			{
				float minCost = float.MaxValue;
				int iMin = -1, jMin = -1;
				for (var i = 0; i < count; ++i)
				{
					AABB aabbi = m_nodes[nodes[i]].aabb;

					for (int j = i + 1; j < count; ++j)
					{
						AABB aabbj = m_nodes[nodes[j]].aabb;
						var b = new AABB();
						b.Combine(in aabbi, in aabbj);
						float cost = b.GetPerimeter();
						if (cost < minCost)
						{
							iMin = i;
							jMin = j;
							minCost = cost;
						}
					}
				}

				int index1 = nodes[iMin];
				int index2 = nodes[jMin];
				TreeNode child1 = m_nodes[index1];
				TreeNode child2 = m_nodes[index2];

				int parentIndex = AllocateNode();
				TreeNode parent = m_nodes[parentIndex];
				parent.child1 = index1;
				parent.child2 = index2;
				parent.height = 1 + Math.Max(child1.height, child2.height);
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
			for (var i = 0; i < m_nodeCapacity; ++i)
			{
				m_nodes[i].aabb.lowerBound -= newOrigin;
				m_nodes[i].aabb.upperBound -= newOrigin;
			}
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public object GetUserData(int proxyId) => m_nodes[proxyId].userData;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public bool WasMoved(int proxyId) => m_nodes[proxyId].moved;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void ClearMoved(int proxyId)
		{
			m_nodes[proxyId].moved = false;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public AABB GetFatAABB(int proxyId) => m_nodes[proxyId].aabb;

		public void Query(Func<int, bool> queryCallback, in AABB aabb)
		{
			var stack = new Stack<int>(256);
			stack.Push(m_root);

			while (stack.Count > 0)
			{
				int nodeId = stack.Pop();
				if (nodeId == -1)
				{
					continue;
				}

				TreeNode node = m_nodes[nodeId];

				if (Collision.TestOverlap(node.aabb, aabb))
				{
					if (node.IsLeaf())
					{
						bool proceed = queryCallback(nodeId);
						if (proceed == false)
						{
							return;
						}
					}
					else
					{
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
			var abs_v = Vector2.Abs(v);

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

			var stack = new Stack<int>(256);
			stack.Push(m_root);

			while (stack.Count > 0)
			{
				int nodeId = stack.Pop();
				if (nodeId == -1)
				{
					continue;
				}

				TreeNode node = m_nodes[nodeId];

				if (Collision.TestOverlap(node.aabb, segmentAABB) == false)
				{
					continue;
				}

				// Separating axis for segment (Gino, p80).
				// |dot(v, p1 - c)| > dot(|v|, h)
				Vector2 c = node.aabb.GetCenter();
				Vector2 h = node.aabb.GetExtents();
				float separation = MathF.Abs(Vector2.Dot(v, p1 - c)) - Vector2.Dot(abs_v, h);
				if (separation > 0.0f)
				{
					continue;
				}

				if (node.IsLeaf())
				{
					RayCastInput subInput;
					subInput.p1 = input.p1;
					subInput.p2 = input.p2;
					subInput.maxFraction = maxFraction;

					float value = RayCastCallback(subInput, nodeId);

					if (value == 0.0f)
						// The client has terminated the ray cast.
					{
						return;
					}

					if (value > 0.0f)
					{
						// Update segment bounding box.
						maxFraction = value;
						Vector2 t = p1 + maxFraction * (p2 - p1);
						segmentAABB.lowerBound = Vector2.Min(p1, t);
						segmentAABB.upperBound = Vector2.Max(p1, t);
					}
				}
				else
				{
					stack.Push(node.child1);
					stack.Push(node.child2);
				}
			}
		}
	}
}