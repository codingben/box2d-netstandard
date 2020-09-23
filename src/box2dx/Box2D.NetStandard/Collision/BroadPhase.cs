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


/*
This broad phase uses the Sweep and Prune algorithm as described in:
Collision Detection in Interactive 3D Environments by Gino van den Bergen
Also, some ideas, such as using integral values for fast compares comes from
Bullet (http:/www.bulletphysics.com).
*/

// Notes:
// - we use bound arrays instead of linked lists for cache coherence.
// - we use quantized integral values for fast compares.
// - we use short indices rather than pointers to save memory.
// - we use a stabbing count for fast overlap queries (less than order N).
// - we also use a time stamp on each proxy to speed up the registration of
//   overlap query results.
// - where possible, we compare bound indices instead of values to reduce
//   cache misses (TODO_ERIN).
// - no broadphase is perfect and neither is this one: it is not great for huge
//   worlds (use a multi-SAP instead), it is not great for large objects.

#define ALLOWUNSAFE
//#define TARGET_FLOAT32_IS_FIXED

using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Box2D.NetStandard.Collision
{
	internal class BroadPhase
	{
		private readonly DynamicTree m_tree;
		private int[] m_moveBuffer;
		private int m_moveCapacity;
		private int m_moveCount;
		private Pair[] m_pairBuffer;
		private int m_pairCapacity;
		private int m_pairCount;

		private int m_proxyCount;
		private int m_queryProxyId;

		public BroadPhase()
		{
			m_proxyCount = 0;

			m_pairCapacity = 16;
			m_pairCount = 0;
			m_pairBuffer = new Pair[m_pairCapacity];

			m_moveCapacity = 16;
			m_moveCount = 0;
			m_moveBuffer = new int[m_moveCapacity];

			m_tree = new DynamicTree();
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal object GetUserData(int proxyId) => m_tree.GetUserData(proxyId);

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public bool TestOverlap(int proxyIdA, int proxyIdB)
		{
			AABB aabbA = m_tree.GetFatAABB(proxyIdA);
			AABB aabbB = m_tree.GetFatAABB(proxyIdB);
			return Collision.TestOverlap(aabbA, aabbB);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public AABB GetFatAABB(int proxyId) => m_tree.GetFatAABB(proxyId);

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public int GetProxyCount() => m_proxyCount;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public int GetTreeHeight() => m_tree.Height;

		public void UpdatePairs(Action<object, object> AddPair)
		{
			m_pairCount = 0;

			for (var i = 0; i < m_moveCount; ++i)
			{
				m_queryProxyId = m_moveBuffer[i];
				if (m_queryProxyId == -1)
				{
					continue;
				}

				AABB fatAABB = m_tree.GetFatAABB(m_queryProxyId);

				m_tree.Query(QueryCallback, fatAABB);
			}

			for (var i = 0; i < m_pairCount; ++i)
			{
				Pair primaryPair = m_pairBuffer[i];
				object userDataA = m_tree.GetUserData(primaryPair.proxyIdA);
				object userDataB = m_tree.GetUserData(primaryPair.proxyIdB);
				AddPair(userDataA, userDataB);
			}

			for (var i = 0; i < m_moveCount; ++i)
			{
				int proxyId = m_moveBuffer[i];
				if (proxyId == -1)
				{
					continue;
				}

				m_tree.ClearMoved(proxyId);
			}

			m_moveCount = 0;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void Query(Func<int, bool> queryCallback, in AABB aabb)
		{
			m_tree.Query(queryCallback, in aabb);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void RayCast(Func<RayCastInput, int, float> RayCastCallback, in RayCastInput input)
		{
			m_tree.RayCast(RayCastCallback, in input);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void ShiftOrigin(in Vector2 newOrigin)
		{
			m_tree.ShiftOrigin(in newOrigin);
		}

		public int CreateProxy(in AABB aabb, object userData)
		{
			int proxyId = m_tree.CreateProxy(aabb, userData);
			++m_proxyCount;
			BufferMove(proxyId);
			return proxyId;
		}

		public void DestroyProxy(int proxyId)
		{
			UnBufferMove(proxyId);
			--m_proxyCount;
			m_tree.DestroyProxy(proxyId);
		}

		// Call MoveProxy as many times as you like, then when you are done
		// call Commit to finalize the proxy pairs (for your time step).
		public void MoveProxy(int proxyId, in AABB aabb, in Vector2 displacement)
		{
			bool buffer = m_tree.MoveProxy((DynamicTree.Proxy)proxyId, aabb, displacement);
			if (buffer)
			{
				BufferMove(proxyId);
			}
		}

		internal void TouchProxy(int proxyId)
		{
			BufferMove(proxyId);
		}


		private void BufferMove(int proxyId)
		{
			if (m_moveCount == m_moveCapacity)
			{
				int[] oldBuffer = m_moveBuffer;
				m_moveCapacity *= 2;
				m_moveBuffer = new int[m_moveCapacity];
				Array.Copy(oldBuffer, m_moveBuffer, m_moveCount);
			}

			m_moveBuffer[m_moveCount] = proxyId;
			++m_moveCount;
		}

		private void UnBufferMove(int proxyId)
		{
			for (var i = 0; i < m_moveCount; ++i)
			{
				if (m_moveBuffer[i] == proxyId)
				{
					m_moveBuffer[i] = -1;
				}
			}
		}

		private bool QueryCallback(int proxyId)
		{
			// A proxy cannot form a pair with itself.
			if (proxyId == m_queryProxyId)
			{
				return true;
			}

			bool moved = m_tree.WasMoved(proxyId);
			if (moved && proxyId > m_queryProxyId)
				// Both proxies are moving. Avoid duplicate pairs.
			{
				return true;
			}

			// Grow the pair buffer as needed.
			if (m_pairCount == m_pairCapacity)
			{
				Pair[] oldBuffer = m_pairBuffer;
				m_pairCapacity = m_pairCapacity + (m_pairCapacity >> 1);
				m_pairBuffer = new Pair[m_pairCapacity];
				Array.Copy(oldBuffer, m_pairBuffer, m_pairCount);
			}

			m_pairBuffer[m_pairCount].proxyIdA = Math.Min(proxyId, m_queryProxyId);
			m_pairBuffer[m_pairCount].proxyIdB = Math.Max(proxyId, m_queryProxyId);
			++m_pairCount;

			return true;
		}
	}
}