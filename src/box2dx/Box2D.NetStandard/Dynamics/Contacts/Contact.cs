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

using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Collision;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Fixtures;
using Box2D.NetStandard.Dynamics.World.Callbacks;

namespace Box2D.NetStandard.Dynamics.Contacts {
  /// <summary>
  /// The class manages contact between two shapes. A contact exists for each overlapping
  /// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
  /// that has no contact points.
  /// </summary>
  public abstract class Contact {
    private static ContactRegister[][] s_registers =
      new ContactRegister[(int) ShapeType.TypeCount][ /*(int)ShapeType.ShapeTypeCount*/];

    private static bool s_initialized;

    internal CollisionFlags m_flags;

    // World pool and list pointers.
    internal Contact m_prev;
    internal Contact m_next;

    // Nodes for connecting bodies.
    internal ContactEdge m_nodeA;
    internal ContactEdge m_nodeB;

    internal Fixture m_fixtureA;
    internal Fixture m_fixtureB;

    internal int m_indexA;
    internal int m_indexB;

    internal Manifold m_manifold = new Manifold();

    private int m_toiCount;
    internal float _toi;

    private float m_friction;
    private float m_restitution;

    private float m_tangentSpeed;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Manifold GetManifold() => m_manifold;

    internal Manifold Manifold {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => m_manifold;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void GetWorldManifold(out WorldManifold worldManifold) {
      Body bodyA = m_fixtureA.Body;
      Body bodyB = m_fixtureB.Body;
      Shape shapeA = m_fixtureA.Shape;
      Shape shapeB = m_fixtureB.Shape;

      worldManifold = new WorldManifold();
      worldManifold.Initialize(m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetEnabled(bool flag) {
      if (flag) {
        m_flags |= CollisionFlags.Enabled;
      }
      else {
        m_flags &= ~CollisionFlags.Enabled;
      }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsEnabled() => (m_flags & CollisionFlags.Enabled) == CollisionFlags.Enabled;

    public bool Enabled {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => IsEnabled();
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      set => SetEnabled(value);
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsTouching() => (m_flags & CollisionFlags.Touching) == CollisionFlags.Touching;

    public bool Touching {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => IsTouching();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Contact GetNext() => m_next;

    public Contact Next {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => GetNext();
    }
    
    /// <summary>
    /// Get the first fixture in this contact.
    /// </summary>
    public Fixture FixtureA {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => m_fixtureA;
    }

    /// <summary>
    /// Get the second fixture in this contact.
    /// </summary>
    public Fixture FixtureB {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => m_fixtureB;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixture GetFixtureA() => FixtureA;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixture GetFixtureB() => FixtureB;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void FlagForFiltering() => m_flags |= CollisionFlags.Filter;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetFriction() => m_friction;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetFriction(float friction) => m_friction = friction;

    public float Friction {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => GetFriction();
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      set => SetFriction(value);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ResetFriction() => m_friction = Settings.MixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetRestitution(float restitution) => m_restitution = restitution;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetRestitution() => m_restitution;

    public float Restitution {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => GetRestitution();
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      set => SetRestitution(value);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ResetRestitution() =>
      m_restitution = Settings.MixRestitution(m_fixtureA.m_restitution, m_fixtureB.m_restitution);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetTangentSpeed(float speed) => m_tangentSpeed = speed;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetTangentSpeed() => m_tangentSpeed;

    public float TangentSpeed {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => GetTangentSpeed();
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      set => SetTangentSpeed(value);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GetChildIndexA() => m_indexA;

    public int ChildIndexA {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => GetChildIndexA();
    }
    
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GetChildIndexB() => m_indexB;

    public int ChildIndexB {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => GetChildIndexB();
    }
    
    
    public   float                _friction;
    public   float                _restitution;
    public   float                _tangentSpeed;
    private  int                  _indexA;
    private  int                  _indexB;
    internal int                  _toiCount;

    internal abstract void Evaluate(out Manifold manifold, in Transform xfA, in Transform xfB);
    
    public Contact(Fixture fA, int indexA, Fixture fB, int indexB) {
      m_flags = CollisionFlags.Enabled;

      m_fixtureA = fA;
      m_fixtureB = fB;

      _indexA = indexA;
      _indexB = indexB;

      m_manifold.pointCount = 0;

      m_prev = null;
      m_next = null;

      m_nodeA = new ContactEdge();
      m_nodeB = new ContactEdge();

      _toiCount = 0;

      _friction    = Settings.MixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);
      _restitution = Settings.MixRestitution(m_fixtureA.Restitution, m_fixtureB.Restitution);

      m_tangentSpeed = 0f;
    }


    public static void AddType(ContactCreateFcn createFcn, ContactDestroyFcn destoryFcn,
      ShapeType                                 type1,     ShapeType         type2) {
      //Debug.Assert(ShapeType.Invalid < type1 && type1 < ShapeType.TypeCount);
      //Debug.Assert(ShapeType.Invalid < type2 && type2 < ShapeType.TypeCount);

      if (s_registers[(int) type1] == null)
        s_registers[(int) type1] = new ContactRegister[(int) ShapeType.TypeCount];

      s_registers[(int) type1][(int) type2].CreateFcn  = createFcn;
      s_registers[(int) type1][(int) type2].DestroyFcn = destoryFcn;
      s_registers[(int) type1][(int) type2].Primary    = true;

      if (type1 != type2) {
        s_registers[(int) type2][(int) type1].CreateFcn  = createFcn;
        s_registers[(int) type2][(int) type1].DestroyFcn = destoryFcn;
        s_registers[(int) type2][(int) type1].Primary    = false;
      }
    }

    public static void InitializeRegisters() {
      AddType(CircleContact.Create, CircleContact.Destroy, ShapeType.Circle, ShapeType.Circle);
      AddType(PolyAndCircleContact.Create, PolyAndCircleContact.Destroy, ShapeType.Polygon, ShapeType.Circle);
      AddType(PolygonContact.Create, PolygonContact.Destroy, ShapeType.Polygon, ShapeType.Polygon);
      AddType(EdgeAndCircleContact.Create, EdgeAndCircleContact.Destroy, ShapeType.Edge, ShapeType.Circle);
      AddType(EdgeAndPolygonContact.Create, EdgeAndPolygonContact.Destroy, ShapeType.Edge, ShapeType.Polygon);
#if CHAINSHAPE
      AddType(ChainAndCircleContact.Create, ChainAndCircleContact.Destroy, ShapeType.Chain, ShapeType.Circle);
      AddType(ChainAndPolygonContact.Create, ChainAndPolygonContact.Destroy, ShapeType.Chain,
              ShapeType.Polygon);
#endif
    }

    public static Contact Create(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB) {
      if (s_initialized == false) {
        InitializeRegisters();
        s_initialized = true;
      }

      ShapeType type1 = fixtureA.Type;
      ShapeType type2 = fixtureB.Type;

      //Debug.Assert(ShapeType.Invalid < type1 && type1 < ShapeType.TypeCount);
      //Debug.Assert(ShapeType.Invalid < type2 && type2 < ShapeType.TypeCount);

      ContactCreateFcn createFcn = s_registers[(int) type1][(int) type2].CreateFcn;
      if (createFcn != null) {
        if (s_registers[(int) type1][(int) type2].Primary) {
          return createFcn(fixtureA, fixtureB);
        }
        else {
          return createFcn(fixtureB, fixtureA);
        }
      }
      else {
        return null;
      }
    }

    public static void Destroy(ref Contact contact) {
      //Debug.Assert(s_initialized == true);

      Fixture fixtureA = contact.m_fixtureA;
      Fixture fixtureB = contact.m_fixtureB;
      
      if (contact.m_manifold.pointCount > 0 &&
          fixtureA.IsSensor()==false &&
          fixtureB.IsSensor()==false) {
        fixtureA.Body.SetAwake(true);
        fixtureB.Body.SetAwake(true);
      }

      ShapeType typeA = fixtureA.Type;
      ShapeType typeB = fixtureB.Type;

      //Debug.Assert(ShapeType.Invalid < typeA && typeA < ShapeType.TypeCount);
      //Debug.Assert(ShapeType.Invalid < typeB && typeB < ShapeType.TypeCount);

      ContactDestroyFcn destroyFcn = s_registers[(int) typeA][(int) typeB].DestroyFcn;
      destroyFcn(ref contact);
    }

    public void Update(ContactListener listener) {
      Manifold oldManifold = m_manifold;

      // Re-enable this contact.
      m_flags |= CollisionFlags.Enabled;

      bool touching    = false;
      bool wasTouching = m_flags.HasFlag(CollisionFlags.Touching);

      bool sensorA = m_fixtureA.IsSensor();
      bool sensorB = m_fixtureB.IsSensor();
      bool sensor  = sensorA || sensorB;

      Body      bodyA = m_fixtureA.Body;
      Body      bodyB = m_fixtureB.Body;
      Transform xfA   = bodyA.GetTransform();
      Transform xfB   = bodyB.GetTransform();

      // Is this contact a sensor?
      if (sensor) {
        Shape shapeA = m_fixtureA.Shape;
        Shape shapeB = m_fixtureB.Shape;
        touching = TestOverlap(shapeA, _indexA, shapeB, _indexB, xfA, xfB);

        // Sensors don't generate manifolds.
        m_manifold.pointCount = 0;
      }
      else {
        Evaluate(out m_manifold, xfA, xfB);
        touching = m_manifold.pointCount > 0;

        // Match old contact ids to new contact ids and copy the
        // stored impulses to warm start the solver.
        for (int i = 0; i < m_manifold.pointCount; ++i) {
          ManifoldPoint mp2 = m_manifold.points[i];
          mp2.normalImpulse  = 0.0f;
          mp2.tangentImpulse = 0.0f;
          ContactID id2 = mp2.id;

          for (int j = 0; j < oldManifold.pointCount; ++j) {
            ManifoldPoint mp1 = oldManifold.points[j];

            if (mp1.id.key == id2.key) {
              mp2.normalImpulse  = mp1.normalImpulse;
              mp2.tangentImpulse = mp1.tangentImpulse;
              break;
            }
          }
        }

        if (touching != wasTouching) {
          bodyA.SetAwake(true);
          bodyB.SetAwake(true);
        }
      }

      if (touching) {
        m_flags |= CollisionFlags.Touching;
      }
      else {
        m_flags &= ~CollisionFlags.Touching;
      }

      if (listener != null) {
        if (wasTouching == false && touching == true) {
          listener.BeginContact(this);
        }

        if (wasTouching == true && touching == false) {
          listener.EndContact(this);
        }

        if (sensor == false && touching==true) {
          listener.PreSolve(this, oldManifold);
        }
      }
    }

    private bool TestOverlap(in Shape     shapeA, int          indexA,
      in                        Shape     shapeB, int          indexB,
      in                        Transform xfA,    in Transform xfB) {
      DistanceInput input = new DistanceInput();
      input.proxyA.Set(shapeA, indexA);
      input.proxyB.Set(shapeB, indexB);
      input.transformA = xfA;
      input.transformB = xfB;
      input.useRadii   = true;

      SimplexCache cache = new SimplexCache();
      cache.count = 0;

      Distance(out DistanceOutput output, cache, in input);

      return output.distance < 10.0f * Settings.FLT_EPSILON;
    }

    internal static void Distance(out DistanceOutput output, SimplexCache cache, in DistanceInput input) {
      output = new DistanceOutput();
      
      DistanceProxy proxyA = input.proxyA;
      DistanceProxy proxyB = input.proxyB;

      Transform transformA = input.transformA;
      Transform transformB = input.transformB;

      // Initialize the simplex.
      Simplex simplex = new Simplex();
      simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);

      // Get simplex vertices as an array.
      SimplexVertex[] vertices   = simplex.m_v;
      const int           k_maxIters = 20;

      // These store the vertices of the last simplex so that we
      // can check for duplicates and prevent cycling.
      int[] saveA     = new int[3], saveB = new int[3];
      int   saveCount = 0;

      // Main iteration loop.
      int iter = 0;
      while (iter < k_maxIters) {
        // Copy simplex so we can identify duplicates.
        saveCount = simplex.m_count;
        for (int i = 0; i < saveCount; ++i) {
          saveA[i] = vertices[i].indexA;
          saveB[i] = vertices[i].indexB;
        }

        switch (simplex.m_count) {
          case 1:
            break;

          case 2:
            simplex.Solve2();
            break;

          case 3:
            simplex.Solve3();
            break;

          default:
            //Debug.Assert(false);
            break;
        }

        // If we have 3 points, then the origin is in the corresponding triangle.
        if (simplex.m_count == 3) {
          break;
        }

        // Get search direction.
        Vector2 d = simplex.GetSearchDirection();

        // Ensure the search direction is numerically fit.
        if (d.LengthSquared() < Settings.FLT_EPSILON_SQUARED) {
          // The origin is probably contained by a line segment
          // or triangle. Thus the shapes are overlapped.

          // We can't return zero here even though there may be overlap.
          // In case the simplex is a point, segment, or triangle it is difficult
          // to determine if the origin is contained in the CSO or very close to it.
          break;
        }

        // Compute a tentative new simplex vertex using support points.
        SimplexVertex vertex = vertices[simplex.m_count];
        vertex.indexA = proxyA.GetSupport(Math.MulT(transformA.q, -d));
        vertex.wA     = Math.Mul(transformA, proxyA.GetVertex(vertex.indexA));
        vertex.indexB = proxyB.GetSupport(Math.MulT(transformB.q, d));
        vertex.wB     = Math.Mul(transformB, proxyB.GetVertex(vertex.indexB));
        vertex.w      = vertex.wB - vertex.wA;

        // Iteration count is equated to the number of support point calls.
        ++iter;
        //++b2_gjkIters;

        // Check for duplicate support points. This is the main termination criteria.
        bool duplicate = false;
        for (int i = 0; i < saveCount; ++i) {
          if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i]) {
            duplicate = true;
            break;
          }
        }

        // If we found a duplicate support point we must exit to avoid cycling.
        if (duplicate) {
          break;
        }

        // New vertex is ok and needed.
        ++simplex.m_count;
      }

      //_gjkMaxIters = b2Max(b2_gjkMaxIters, iter);

      // Prepare output.
      simplex.GetWitnessPoints(out output.pointA, out output.pointB);
      output.distance   = Vector2.Distance(output.pointA, output.pointB);
      output.iterations = iter;

      // Cache the simplex.
      simplex.WriteCache(cache);

      // Apply radii if requested.
      if (input.useRadii) {
        float rA = proxyA._radius;
        float rB = proxyB._radius;

        if (output.distance > rA + rB && output.distance > Settings.FLT_EPSILON) {
          // Shapes are still no overlapped.
          // Move the witness points to the outer surface.
          output.distance -= rA + rB;
          Vector2 normal = output.pointB - output.pointA;
          normal        =  Vector2.Normalize(normal);
          output.pointA += rA * normal;
          output.pointB -= rB * normal;
        }
        else {
          // Shapes are overlapped when radii are considered.
          // Move the witness points to the middle.
          Vector2 p = 0.5f * (output.pointA + output.pointB);
          output.pointA   = p;
          output.pointB   = p;
          output.distance = 0.0f;
        }
      }
    }


  }
}