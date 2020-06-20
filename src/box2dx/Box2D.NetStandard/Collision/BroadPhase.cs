/*
  Box2DX Copyright (c) 2008 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
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

namespace Box2D.NetStandard.Collision {
#warning "CAS"


  internal class BroadPhase {
    private DynamicTree _tree;

    internal int _proxyCount;

    private int[] _moveBuffer;
    private int   _moveCapacity;
    private int   _moveCount;

    private Pair[] _pairBuffer;
    private int    _pairCapacity;
    private int    _pairCount;

    private int _queryProxyId;

    public BroadPhase() {
      _proxyCount = 0;

      _pairCapacity = 16;
      _pairCount    = 0;
      _pairBuffer   = new Pair[_pairCapacity];

      _moveCapacity = 16;
      _moveCount    = 0;
      _moveBuffer   = new int[_moveCapacity];

      _tree = new DynamicTree();
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal object GetUserData(int proxyId) => _tree.GetUserData(proxyId);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TestOverlap(int proxyIdA, int proxyIdB) {
      AABB aabbA = _tree.GetFatAABB(proxyIdA);
      AABB aabbB = _tree.GetFatAABB(proxyIdB);
      return Collision.TestOverlap(aabbA, aabbB);
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public AABB GetFatAABB(int proxyId) => _tree.GetFatAABB(proxyId);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GetProxyCount() => _proxyCount;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GetTreeHeight() => _tree.GetHeight();

    public void UpdatePairs(Action<object, object> AddPair) {
      _pairCount = 0;

      for (int i = 0; i < _moveCount; ++i) {
        _queryProxyId = _moveBuffer[i];
        if (_queryProxyId == -1) continue;

        AABB fatAABB = _tree.GetFatAABB(_queryProxyId);

        _tree.Query(QueryCallback, fatAABB);
      }

      for (int i = 0; i < _pairCount; ++i) {
        Pair   primaryPair = _pairBuffer[i];
        object userDataA   = _tree.GetUserData(primaryPair.proxyIdA);
        object userDataB   = _tree.GetUserData(primaryPair.proxyIdB);
        AddPair(userDataA, userDataB);
      }

      for (int i = 0; i < _moveCount; ++i) {
        int proxyId = _moveBuffer[i];
        if (proxyId == -1) continue;
        _tree.ClearMoved(proxyId);
      }

      _moveCount = 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Query(Func<int, bool> queryCallback, in AABB aabb) => _tree.Query(queryCallback, in aabb);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void RayCast(Func<RayCastInput, int, float> RayCastCallback, in RayCastInput input) =>
      _tree.RayCast(RayCastCallback, in input);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ShiftOrigin(in Vector2 newOrigin) => _tree.ShiftOrigin(in newOrigin);

    public int CreateProxy(in AABB aabb, object userData) {
      int proxyId = _tree.CreateProxy(aabb, userData);
      ++_proxyCount;
      BufferMove(proxyId);
      return proxyId;
    }

    public void DestroyProxy(int proxyId) {
      UnBufferMove(proxyId);
      --_proxyCount;
      _tree.DestroyProxy(proxyId);
    }

    // Call MoveProxy as many times as you like, then when you are done
    // call Commit to finalized the proxy pairs (for your time step).
    public void MoveProxy(int proxyId, in AABB aabb, in Vector2 displacement) {
      bool buffer = _tree.MoveProxy(proxyId, aabb, displacement);
      if (buffer) {
        BufferMove(proxyId);
      }
    }

    internal void TouchProxy(int proxyId) {
      BufferMove(proxyId);
    }


    public void BufferMove(int proxyId) {
      if (_moveCount == _moveCapacity) {
        int[] oldBuffer = _moveBuffer;
        _moveCapacity *= 2;
        _moveBuffer   =  new int[_moveCapacity];
        Array.Copy(oldBuffer, _moveBuffer, _moveCount);
      }

      _moveBuffer[_moveCount] = proxyId;
      ++_moveCount;
    }

    public void UnBufferMove(int proxyId) {
      for (int i = 0; i < _moveCount; ++i) {
        if (_moveBuffer[i] == proxyId) {
          _moveBuffer[i] = -1;
        }
      }
    }

    public bool QueryCallback(int proxyId) {
      // A proxy cannot form a pair with itself.
      if (proxyId == _queryProxyId) {
        return true;
      }

      bool moved = _tree.WasMoved(proxyId);
      if (moved && proxyId > _queryProxyId) {
        // Both proxies are moving. Avoid duplicate pairs.
        return true;
      }

      // Grow the pair buffer as needed.
      if (_pairCount == _pairCapacity) {
        Pair[] oldBuffer = _pairBuffer;
        _pairCapacity = _pairCapacity + (_pairCapacity >> 1);
        _pairBuffer   = new Pair[_pairCapacity];
        Array.Copy(oldBuffer, _pairBuffer, _pairCount);
      }

      _pairBuffer[_pairCount].proxyIdA = System.Math.Min(proxyId, _queryProxyId);
      _pairBuffer[_pairCount].proxyIdB = System.Math.Max(proxyId, _queryProxyId);
      ++_pairCount;

      return true;
    }
    
    
    


  }
}