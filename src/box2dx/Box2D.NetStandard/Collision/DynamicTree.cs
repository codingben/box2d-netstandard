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
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using Box2D.NetStandard.Common;
using Math = System.Math;
using Proxy = System.Int32;

namespace Box2D.NetStandard.Collision
{
	/// <summary>
	///
	/// </summary>
	/// <typeparam name="T"></typeparam>
	internal sealed class DynamicTree
	{
		private const Proxy ProxyFree = -1;
		
		private struct Node
		{
			public AABB Aabb;
			public Proxy Parent;
			public Proxy Child1;
			public Proxy Child2;

			public object UserData;

			public int Height;
			public bool Moved;

			public bool IsLeaf
			{
				[MethodImpl(MethodImplOptions.AggressiveInlining)]
				get => Child2 == ProxyFree;
			}

			public bool IsFree
			{
				[MethodImpl(MethodImplOptions.AggressiveInlining)]
				get => Height == -1;
			}

			public override string ToString()
				=> $@"Parent: {(Parent == ProxyFree ? "None" : Parent.ToString())}, {
					(IsLeaf
						 ? Height == 0
							   ? $"Leaf: {UserData}"
							   : $"Leaf (invalid height of {Height}): {UserData}"
						 : IsFree
							 ? "Free"
							 : $"Branch at height {Height}, children: {Child1} and {Child2}")}";
		}

		public int Capacity => _nodes.Length;
		private Node[] _nodes;
		private Proxy _root;
		private Proxy _freeNodes;
		private int _nodeCount;

		public int Height
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => _root == ProxyFree ? 0 : _nodes[_root].Height;
		}

		public int NodeCount => _nodeCount;

		public int MaxBalance
		{
			[MethodImpl(MethodImplOptions.NoInlining)]
			get {
				var maxBal = 0;

				for (var i = 0; i < Capacity; ++i)
				{
					ref var node = ref _nodes[i];
					if (node.Height <= 1)
					{
						continue;
					}

					ref var child1Node = ref _nodes[node.Child1];
					ref var child2Node = ref _nodes[node.Child2];

					var bal = Math.Abs(child2Node.Height - child1Node.Height);
					maxBal = Math.Max(maxBal, bal);
				}

				return maxBal;
			}
		}

		public float AreaRatio
		{
			[MethodImpl(MethodImplOptions.NoInlining)]
			get {
				if (_root == ProxyFree)
				{
					return 0;
				}

				ref var rootNode = ref _nodes[_root];
				var rootPeri = rootNode.Aabb.GetPerimeter();

				var totalPeri = 0f;

				for (var i = 0; i < Capacity; ++i)
				{
					ref var node = ref _nodes[i];
					if (node.Height < 0)
					{
						continue;
					}

					totalPeri += node.Aabb.GetPerimeter();
				}

				return totalPeri / rootPeri;
			}
		}

		private static int GrowthFunc(int x) => x + 256;

		private const float AABBExtendSize = 1f / 32;

		private const float AABBMultiplier = 2f;

		public DynamicTree()
		{
			_root = ProxyFree;
			_nodes = new Node[256];

			// Build a linked list for the free list.
			ref Node node = ref _nodes[0];
			var l = Capacity - 1;

			for (var i = 0; i < l; i++, node = ref _nodes[i])
			{
				node.Parent = (Proxy) (i + 1);
				node.Height = -1;
			}

			ref var lastNode = ref _nodes[^1];

			lastNode.Parent = ProxyFree;
			lastNode.Height = -1;
		}

		/// <summary>Allocate a node from the pool. Grow the pool if necessary.</summary>
		/// <remarks>
		///     If allocation occurs, references to <see cref="Node" />s will be invalid.
		/// </remarks>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private ref Node AllocateNode(out Proxy proxy)
		{
			// Expand the node pool as needed.
			if (_freeNodes == ProxyFree)
			{
				// Separate method to aid inlining since this is a cold path.
				Expand();
			}

			// Peel a node off the free list.
			var alloc = _freeNodes;
			ref var allocNode = ref _nodes[alloc];
			Assert(allocNode.IsFree);
			_freeNodes = allocNode.Parent;
			Assert(_freeNodes == -1 || _nodes[_freeNodes].IsFree);
			allocNode.Parent = ProxyFree;
			allocNode.Child1 = ProxyFree;
			allocNode.Child2 = ProxyFree;
			allocNode.Height = 0;
			++_nodeCount;
			proxy = alloc;
			return ref allocNode;

			void Expand()
			{
				Assert(_nodeCount == Capacity);

				// The free list is empty. Rebuild a bigger pool.
				var newNodeCap = GrowthFunc(Capacity);

				if (newNodeCap <= Capacity)
				{
					throw new InvalidOperationException(
					                                    "Growth function returned invalid new capacity, must be greater than current capacity.");
				}

				var oldNodes = _nodes;

				_nodes = new Node[newNodeCap];

				Array.Copy(oldNodes, _nodes, _nodeCount);

				// Build a linked list for the free list. The parent
				// pointer becomes the "next" pointer.
				var l = _nodes.Length - 1;
				ref Node node = ref _nodes[_nodeCount];
				for (var i = _nodeCount; i < l; ++i, node = ref _nodes[i])
				{
					node.Parent = (Proxy) (i + 1);
					node.Height = -1;
				}

				ref var lastNode = ref _nodes[l];
				lastNode.Parent = ProxyFree;
				lastNode.Height = -1;
				_freeNodes = (Proxy) _nodeCount;
			}
		}

		/// <summary>
		///     Return a node to the pool.
		/// </summary>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private void FreeNode(Proxy proxy)
		{
			ref var node = ref _nodes[proxy];
			node.Parent = _freeNodes;
			node.Height = -1;
#if DEBUG_DYNAMIC_TREE
            node.Child1 = ProxyFree;
            node.Child2 = ProxyFree;
#endif
			node.UserData = default;
			_freeNodes = proxy;
			--_nodeCount;
		}

		/// <summary>
		///     Create a proxy in the tree as a leaf node.
		/// </summary>
		public Proxy CreateProxy(in AABB aabb, object userData)
		{
			ref var proxy = ref AllocateNode(out var proxyId);

			// Fatten the aabb.
			proxy.Aabb = aabb.Enlarged(AABBExtendSize);
			proxy.Height = 0;
			proxy.Moved = true;
			proxy.UserData = userData;

			InsertLeaf(proxyId);
			return proxyId;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal void DestroyProxy(Proxy proxy)
		{
			Assert(0 <= proxy && proxy < Capacity);
			Assert(_nodes[proxy].IsLeaf);

			RemoveLeaf(proxy);
			FreeNode(proxy);
		}

		public bool MoveProxy(Proxy proxy, in AABB aabb, Vector2 displacement)
		{
			Assert(0 <= proxy && proxy < Capacity);

			ref var leafNode = ref _nodes[proxy];

			Assert(leafNode.IsLeaf);

			// Extend AABB
			var ext = new Vector2(AABBExtendSize, AABBExtendSize);
			var fatAabb = aabb.Enlarged(AABBExtendSize);

			// Predict AABB movement
			var d = displacement * AABBMultiplier;

			// pls can we make math types mutable this sucks.
			var l = fatAabb.lowerBound.X;
			var b = fatAabb.lowerBound.Y;
			var r = fatAabb.upperBound.X;
			var t = fatAabb.upperBound.Y;

			if (d.X < 0)
			{
				l += d.X;
			}
			else
			{
				r += d.X;
			}

			if (d.Y < 0)
			{
				b += d.Y;
			}
			else
			{
				t += d.Y;
			}

			fatAabb = new AABB(new Vector2(l, b), new Vector2(r, t));

			ref var treeAabb = ref leafNode.Aabb;

			if (treeAabb.Contains(aabb))
			{
				// The tree AABB still contains the object, but it might be too large.
				// Perhaps the object was moving fast but has since gone to sleep.
				// The huge AABB is larger than the new fat AABB.
				Vector2 growAmount = new Vector2(4, 4) * ext;
				var hugeAabb = new AABB(
				                        fatAabb.lowerBound - growAmount,
				                        fatAabb.upperBound + growAmount);

				if (hugeAabb.Contains(treeAabb))
				{
					// The tree AABB contains the object AABB and the tree AABB is
					// not too large. No tree update needed.
					return false;
				}

				// Otherwise the tree AABB is huge and needs to be shrunk
			}

			RemoveLeaf(proxy);

			leafNode.Aabb = fatAabb;

			InsertLeaf(proxy);

			leafNode.Moved = true;

			return true;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public object GetUserData(Proxy proxy)
		{
			return _nodes[proxy].UserData;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public bool WasMoved(Proxy proxy)
		{
			return _nodes[proxy].Moved;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void ClearMoved(Proxy proxy)
		{
			_nodes[proxy].Moved = false;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public AABB GetFatAABB(Proxy proxy)
		{
			return _nodes[proxy].Aabb;
		}

		[MethodImpl(MethodImplOptions.NoInlining)]
		private void RemoveLeaf(Proxy leaf)
		{
			if (leaf == _root)
			{
				_root = ProxyFree;
				return;
			}

			ref var leafNode = ref _nodes[leaf];
			Assert(leafNode.IsLeaf);
			var parent = leafNode.Parent;
			ref var parentNode = ref _nodes[parent];
			var grandParent = parentNode.Parent;
			var sibling = parentNode.Child1 == leaf
				              ? parentNode.Child2
				              : parentNode.Child1;

			ref var siblingNode = ref _nodes[sibling];

			if (grandParent != ProxyFree)
			{
				// Destroy parent and connect sibling to grandParent.
				ref var grandParentNode = ref _nodes[grandParent];
				if (grandParentNode.Child1 == parent)
				{
					grandParentNode.Child1 = sibling;
				}
				else
				{
					grandParentNode.Child2 = sibling;
				}

				siblingNode.Parent = grandParent;
				FreeNode(parent);

				// Adjust ancestor bounds.
				Balance(grandParent);
			}
			else
			{
				_root = sibling;
				siblingNode.Parent = ProxyFree;
				FreeNode(parent);
			}

			Validate();
		}

		private void InsertLeaf(Proxy leaf)
		{
			if (_root == ProxyFree)
			{
				_root = leaf;
				_nodes[_root].Parent = ProxyFree;
				return;
			}

			Validate();

			// Find the best sibling for this node
			ref var leafNode = ref _nodes[leaf];
			ref var leafAabb = ref leafNode.Aabb;

			var index = _root;
#if DEBUG
			var loopCount = 0;
#endif
			for (;;)
			{
#if DEBUG
				Assert(loopCount++ < Capacity * 2);
#endif

				ref var indexNode = ref _nodes[index];
				if (indexNode.IsLeaf) break;

				// assert no loops
				Assert(_nodes[indexNode.Child1].Child1 != index);
				Assert(_nodes[indexNode.Child1].Child2 != index);
				Assert(_nodes[indexNode.Child2].Child1 != index);
				Assert(_nodes[indexNode.Child2].Child2 != index);

				var child1 = indexNode.Child1;
				var child2 = indexNode.Child2;
				ref var child1Node = ref _nodes[child1];
				ref var child2Node = ref _nodes[child2];
				ref var indexAabb = ref indexNode.Aabb;
				var indexPeri = indexAabb.GetPerimeter();
				AABB combinedAabb = default;
				combinedAabb = AABB.Combine(indexAabb, leafAabb);
				var combinedPeri = combinedAabb.GetPerimeter();
				// Cost of creating a new parent for this node and the new leaf
				var cost = 2 * combinedPeri;
				// Minimum cost of pushing the leaf further down the tree
				var inheritCost = 2 * (combinedPeri - indexPeri);

				// Cost of descending into child1
				var cost1 = EstimateCost(leafAabb, child1Node) + inheritCost;
				// Cost of descending into child2
				var cost2 = EstimateCost(leafAabb, child2Node) + inheritCost;

				// Descend according to the minimum cost.
				if (cost < cost1 && cost < cost2)
				{
					break;
				}

				// Descend
				index = cost1 < cost2 ? child1 : child2;
			}

			var sibling = index;

			// Create a new parent.
			ref var newParentNode = ref AllocateNode(out var newParent);
			ref var siblingNode = ref _nodes[sibling];

			var oldParent = siblingNode.Parent;

			newParentNode.Parent = oldParent;
			newParentNode.Aabb = AABB.Combine(leafAabb, siblingNode.Aabb);
			newParentNode.Height = 1 + siblingNode.Height;

			ref var proxyNode = ref _nodes[leaf];
			if (oldParent != ProxyFree)
			{
				// The sibling was not the root.
				ref var oldParentNode = ref _nodes[oldParent];

				if (oldParentNode.Child1 == sibling)
				{
					oldParentNode.Child1 = newParent;
				}
				else
				{
					oldParentNode.Child2 = newParent;
				}

				newParentNode.Child1 = sibling;
				newParentNode.Child2 = leaf;
				siblingNode.Parent = newParent;
				proxyNode.Parent = newParent;
			}
			else
			{
				// The sibling was the root.
				newParentNode.Child1 = sibling;
				newParentNode.Child2 = leaf;
				siblingNode.Parent = newParent;
				proxyNode.Parent = newParent;
				_root = newParent;
			}

			// Walk back up the tree fixing heights and AABBs
			Balance(proxyNode.Parent);

			Validate();
		}


		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private static float EstimateCost(in AABB baseAabb, in Node node)
		{
			var cost = AABB.Combine(baseAabb, node.Aabb).GetPerimeter();

			if (!node.IsLeaf)
			{
				cost -= node.Aabb.GetPerimeter();
			}

			return cost;
		}

		[MethodImpl(MethodImplOptions.NoInlining)]
		private void Balance(Proxy index)
		{
			while (index != ProxyFree)
			{
				index = BalanceStep(index);

				ref var indexNode = ref _nodes[index];

				var child1 = indexNode.Child1;
				var child2 = indexNode.Child2;

				Assert(child1 != ProxyFree);
				Assert(child2 != ProxyFree);

				ref var child1Node = ref _nodes[child1];
				ref var child2Node = ref _nodes[child2];

				indexNode.Height = Math.Max(child1Node.Height, child2Node.Height) + 1;
				indexNode.Aabb = AABB.Combine(child1Node.Aabb, child2Node.Aabb);

				index = indexNode.Parent;
			}

			Validate();
		}

		/// <summary>
		///     Perform a left or right rotation if node A is imbalanced.
		/// </summary>
		/// <returns>The new root index.</returns>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private Proxy BalanceStep(Proxy iA)
		{
			ref var a = ref _nodes[iA];

			if (a.IsLeaf || a.Height < 2)
			{
				return iA;
			}

			var iB = a.Child1;
			var iC = a.Child2;
			Assert(iA != iB);
			Assert(iA != iC);
			Assert(iB != iC);

			ref var b = ref _nodes[iB];
			ref var c = ref _nodes[iC];

			var balance = c.Height - b.Height;

			// Rotate C up
			if (balance > 1)
			{
				var iF = c.Child1;
				var iG = c.Child2;
				Assert(iC != iF);
				Assert(iC != iG);
				Assert(iF != iG);

				ref var f = ref _nodes[iF];
				ref var g = ref _nodes[iG];

				// A <> C

				// this creates a loop ...
				c.Child1 = iA;
				c.Parent = a.Parent;
				a.Parent = iC;

				if (c.Parent == ProxyFree)
				{
					_root = iC;
				}
				else
				{
					ref var cParent = ref _nodes[c.Parent];
					if (cParent.Child1 == iA)
					{
						cParent.Child1 = iC;
					}
					else
					{
						Assert(cParent.Child2 == iA);
						cParent.Child2 = iC;
					}
				}

				// Rotate
				if (f.Height > g.Height)
				{
					c.Child2 = iF;
					a.Child2 = iG;
					g.Parent = iA;
					a.Aabb = AABB.Combine(b.Aabb, g.Aabb);
					c.Aabb = AABB.Combine(a.Aabb, f.Aabb);

					a.Height = Math.Max(b.Height, g.Height) + 1;
					c.Height = Math.Max(a.Height, f.Height) + 1;
				}
				else
				{
					c.Child2 = iG;
					a.Child2 = iF;
					f.Parent = iA;
					a.Aabb = AABB.Combine(b.Aabb, f.Aabb);
					c.Aabb = AABB.Combine(a.Aabb, g.Aabb);

					a.Height = Math.Max(b.Height, f.Height) + 1;
					c.Height = Math.Max(a.Height, g.Height) + 1;
				}

				return iC;
			}

			// Rotate B up
			if (balance < -1)
			{
				var iD = b.Child1;
				var iE = b.Child2;
				Assert(iB != iD);
				Assert(iB != iE);
				Assert(iD != iE);

				ref var d = ref _nodes[iD];
				ref var e = ref _nodes[iE];

				// A <> B

				// this creates a loop ...
				b.Child1 = iA;
				b.Parent = a.Parent;
				a.Parent = iB;

				if (b.Parent == ProxyFree)
				{
					_root = iB;
				}
				else
				{
					ref var bParent = ref _nodes[b.Parent];
					if (bParent.Child1 == iA)
					{
						bParent.Child1 = iB;
					}
					else
					{
						Assert(bParent.Child2 == iA);
						bParent.Child2 = iB;
					}
				}

				// Rotate
				if (d.Height > e.Height)
				{
					b.Child2 = iD;
					a.Child1 = iE;
					e.Parent = iA;
					a.Aabb = AABB.Combine(c.Aabb, e.Aabb);
					b.Aabb = AABB.Combine(a.Aabb, d.Aabb);

					a.Height = Math.Max(c.Height, e.Height) + 1;
					b.Height = Math.Max(a.Height, d.Height) + 1;
				}
				else
				{
					b.Child2 = iE;
					a.Child1 = iD;
					d.Parent = iA;
					a.Aabb = AABB.Combine(c.Aabb, d.Aabb);
					b.Aabb = AABB.Combine(a.Aabb, e.Aabb);

					a.Height = Math.Max(c.Height, d.Height) + 1;
					b.Height = Math.Max(a.Height, e.Height) + 1;
				}

				return iB;
			}

			return iA;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private int ComputeHeight()
			=> ComputeHeight(_root);

		/// <summary>
		///     Compute the height of a sub-tree.
		/// </summary>
		[MethodImpl(MethodImplOptions.NoInlining)]
		private int ComputeHeight(Proxy proxy)
		{
			ref var node = ref _nodes[proxy];
			if (node.IsLeaf)
			{
				return 0;
			}

			return Math.Max(
			                ComputeHeight(node.Child1),
			                ComputeHeight(node.Child2)
			               ) + 1;
		}

		[MethodImpl(MethodImplOptions.NoInlining)]
		public void RebuildBottomUp(int free = 0)
		{
			var proxies = new Proxy[NodeCount + free];
			var count = 0;

			// Build array of leaves. Free the rest.
			for (var i = 0; i < Capacity; ++i)
			{
				ref var node = ref _nodes[i];
				if (node.Height < 0)
				{
					// free node in pool
					continue;
				}

				var proxy = (Proxy) i;
				if (node.IsLeaf)
				{
					node.Parent = ProxyFree;
					proxies[count++] = proxy;
				}
				else
				{
					FreeNode(proxy);
				}
			}

			while (count > 1)
			{
				var minCost = float.MaxValue;

				var iMin = -1;
				var jMin = -1;

				for (var i = 0; i < count; ++i)
				{
					ref var aabbI = ref _nodes[proxies[i]].Aabb;

					for (var j = i + 1; j < count; ++j)
					{
						ref var aabbJ = ref _nodes[proxies[j]].Aabb;

						var cost = AABB.Combine(aabbI, aabbJ).GetPerimeter();

						if (cost >= minCost)
						{
							continue;
						}

						iMin = i;
						jMin = j;
						minCost = cost;
					}
				}

				var child1 = proxies[iMin];
				var child2 = proxies[jMin];

				ref var parentNode = ref AllocateNode(out var parent);
				ref var child1Node = ref _nodes[child1];
				ref var child2Node = ref _nodes[child2];

				parentNode.Child1 = child1;
				parentNode.Child2 = child2;
				parentNode.Height = Math.Max(child1Node.Height, child2Node.Height) + 1;
				parentNode.Aabb = AABB.Combine(child1Node.Aabb, child2Node.Aabb);
				parentNode.Parent = ProxyFree;

				child1Node.Parent = parent;
				child2Node.Parent = parent;

				proxies[jMin] = proxies[count - 1];
				proxies[iMin] = parent;
				--count;
			}

			_root = proxies[0];

			Validate();
		}

		public void ShiftOrigin(in Vector2 newOrigin)
		{
			for (var i = 0; i < _nodes.Length; i++)
			{
				ref var node = ref _nodes[i];
				var lb = node.Aabb.lowerBound;
				var tr = node.Aabb.upperBound;

				node.Aabb = new AABB(lb - newOrigin, tr - newOrigin);
			}
		}

		public void Query(Func<int, bool> queryCallback, in AABB aabb)
		{
			using var stack = new GrowableStack<Proxy>(stackalloc Proxy[256]);
			stack.Push(_root);

			while (stack._count != 0)
			{
				var nodeId = stack.Pop();
				if (nodeId == ProxyFree)
				{
					continue;
				}

				// Skip bounds check with Unsafe.Add().
				var node = _nodes[nodeId];
				if (node.Aabb.Intersects(aabb))
				{
					if (node.IsLeaf)
					{
						var proceed = queryCallback(nodeId);
						if (proceed == false)
						{
							return;
						}
					}
					else
					{
						stack.Push(node.Child1);
						stack.Push(node.Child2);
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
			var absV = Vector2.Abs(v);

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

			using var stack = new GrowableStack<Proxy>(stackalloc Proxy[256]);
			stack.Push(_root);

			while (stack._count != 0)
			{
				var nodeId = stack.Pop();
				if (nodeId == ProxyFree)
				{
					continue;
				}

				// Skip bounds check with Unsafe.Add().
				var node = _nodes[nodeId];
				if (node.Aabb.Intersects(segmentAABB) == false)
				{
					continue;
				}

				// Separating axis for segment (Gino, p80).
				// |dot(v, p1 - c)| > dot(|v|, h)
				var c = node.Aabb.GetCenter();
				var h = node.Aabb.GetExtents();
				var separation = Math.Abs(Vector2.Dot(v, p1 - c)) - Vector2.Dot(absV, h);

				if (separation > 0)
				{
					continue;
				}

				if (node.IsLeaf)
				{
					var subInput = input;

					float value = RayCastCallback(subInput, nodeId);

					if (value == 0f)
					{
						// The client has terminated the ray cast.
						return;
					}

					if (value > 0)
					{
						// Update segment bounding box.
						maxFraction = value;
						var t = p1 + (p2 - p1) * maxFraction;
						segmentAABB = new AABB(
						                       Vector2.Min(p1, t),
						                       Vector2.Max(p1, t));
					}
				}
				else
				{
					stack.Push(node.Child1);
					stack.Push(node.Child2);
				}
			}
		}

		[Conditional("DEBUG")]
		private void Validate()
		{
			Validate(_root);

			var freeCount = 0;
			var freeIndex = _freeNodes;
			while (freeIndex != ProxyFree)
			{
				Assert(0 <= freeIndex);
				Assert(freeIndex < Capacity);
				freeIndex = _nodes[freeIndex].Parent;
				++freeCount;
			}

			Assert(Height == ComputeHeight());

			Assert(NodeCount + freeCount == Capacity);
		}

		[Conditional("DEBUG")]
		private void Validate(Proxy proxy)
		{
			if (proxy == ProxyFree) return;

			ref var node = ref _nodes[proxy];

			if (proxy == _root)
			{
				Assert(node.Parent == ProxyFree);
			}

			var child1 = node.Child1;
			var child2 = node.Child2;

			if (node.IsLeaf)
			{
				Assert(child1 == ProxyFree);
				Assert(child2 == ProxyFree);
				Assert(node.Height == 0);
				return;
			}

			Assert(0 <= child1);
			Assert(child1 < Capacity);
			Assert(0 <= child2);
			Assert(child2 < Capacity);

			ref var child1Node = ref _nodes[child1];
			ref var child2Node = ref _nodes[child2];

			Assert(child1Node.Parent == proxy);
			Assert(child2Node.Parent == proxy);

			var height1 = child1Node.Height;
			var height2 = child2Node.Height;

			var height = 1 + Math.Max(height1, height2);

			Assert(node.Height == height);

			ref var aabb = ref node.Aabb;
			Assert(aabb.Contains(child1Node.Aabb));
			Assert(aabb.Contains(child2Node.Aabb));

			Validate(child1);
			Validate(child2);
		}

		[Conditional("DEBUG_DYNAMIC_TREE")]
		private void ValidateHeight(Proxy proxy)
		{
			if (proxy == ProxyFree)
			{
				return;
			}

			ref var node = ref _nodes[proxy];

			if (node.IsLeaf)
			{
				Assert(node.Height == 0);
				return;
			}

			var child1 = node.Child1;
			var child2 = node.Child2;
			ref var child1Node = ref _nodes[child1];
			ref var child2Node = ref _nodes[child2];

			var height1 = child1Node.Height;
			var height2 = child2Node.Height;

			var height = 1 + Math.Max(height1, height2);

			Assert(node.Height == height);
		}
		
		[Conditional("DEBUG")]
		[DebuggerNonUserCode]
		[DebuggerHidden]
		[DebuggerStepThrough]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void Assert(bool assertion, [CallerMemberName]
			string? member = default,
			[CallerFilePath]
			string? file = default, [CallerLineNumber]
			int line = default)
		{
			if (assertion) return;

			var msg = $"Assertion failure in {member} ({file}:{line})";
			Debug.Print(msg);
			Debugger.Break();
			throw new InvalidOperationException(msg);
		}


		private IEnumerable<(Proxy, Node)> DebugAllocatedNodesEnumerable
		{
			get {
				for (var i = 0; i < _nodes.Length; i++)
				{
					var node = _nodes[i];
					if (!node.IsFree)
					{
						yield return ((Proxy) i, node);
					}
				}
			}
		}

		[DebuggerBrowsable(DebuggerBrowsableState.RootHidden)]
		private (Proxy, Node)[] DebugAllocatedNodes
		{
			get {
				var data = new (Proxy, Node)[NodeCount];
				var i = 0;
				foreach (var x in DebugAllocatedNodesEnumerable)
				{
					data[i++] = x;
				}

				return data;
			}
		}
	}
}