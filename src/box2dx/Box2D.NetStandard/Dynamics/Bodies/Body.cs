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

using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Collision;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Contacts;
using Box2D.NetStandard.Dynamics.Fixtures;
using Box2D.NetStandard.Dynamics.Joints;

namespace Box2D.NetStandard.Dynamics.Bodies {
  /// <summary>
  /// A rigid body. These are created via World.CreateBody.
  /// </summary>
  [DebuggerDisplay("{_userData}")]
  public class Body {
    // No public default constructor
    private Body() { }

    // public
    /// <summary>
    /// Creates a fixture and attaches it to this body. Use this function if you need
    /// to set some fixture parameters, like friction. Otherwise you can create the
    /// fixture directly from a shape.
    /// If the density is non-zero, this function automatically updates the mass of the body.
    /// Contacts are not created until the next time step.
    /// </summary>
    /// <param name="def">the fixture definition.</param>
    /// <warning>This function is locked during callbacks.</warning>
    public Fixture CreateFixture(in FixtureDef def) {
      //Debug.Assert(_world.IsLocked() == false);
      if (_world.IsLocked() == true) {
        return null;
      }

      Fixture fixture = new Fixture();
      fixture.Create(this, def);

      if (HasFlag(BodyFlags.Enabled)) {
        BroadPhase broadPhase = _world._contactManager.m_broadPhase;
        fixture.CreateProxies(broadPhase, _xf);
      }

      fixture.m_next = _fixtureList;
      _fixtureList   = fixture;
      ++_fixtureCount;

      fixture.m_body = this;

      // Adjust mass properties if needed.
      if (fixture.m_density > 0.0f) {
        ResetMassData();
      }

      // Let the world know we have a new fixture. This will cause new contacts
      // to be created at the beginning of the next time step.
      _world._newContacts = true;

      return fixture;
    }

    /// <summary>
    /// Creates a fixture from a shape and attach it to this body.
    /// This is a convenience function. Use b2FixtureDef if you need to set parameters
    /// like friction, restitution, user data, or filtering.
    /// If the density is non-zero, this function automatically updates the mass of the body.
    /// </summary>
    /// <param name="shape">the shape to be cloned.</param>
    /// <param name="density">the shape density (set to zero for static bodies).</param>
    /// <warning>This function is locked during callbacks.</warning>
    public Fixture CreateFixture(in Shape shape, float density) {
      FixtureDef def = new FixtureDef();
      def.shape   = shape;
      def.density = density;

      return CreateFixture(def);
    }

    public void DestroyFixture(Fixture fixture) {
      if (fixture == null) {
        return;
      }

      //Debug.Assert(_world.IsLocked() == false);
      if (_world.IsLocked() == true) {
        return;
      }

      //Debug.Assert(fixture.m_body == this);

      // Remove the fixture from this body's singly linked list.
      //Debug.Assert(_fixtureCount > 0);
      Fixture node  = _fixtureList;
      bool    found = false;
      while (node != null) {
        if (node == fixture) {
          node  = fixture.m_next;
          found = true;
          break;
        }

        node = node.m_next;
      }

      // You tried to remove a shape that is not attached to this body.
      //Debug.Assert(found);

      // Destroy any contacts associated with the fixture.
      ContactEdge edge = _contactList;
      while (edge != null) {
        Contact c = edge.contact;
        edge = edge.next;

        Fixture fixtureA = c.GetFixtureA();
        Fixture fixtureB = c.GetFixtureB();

        if (fixture == fixtureA || fixture == fixtureB) {
          // This destroys the contact and removes it from
          // this body's contact list.
          _world._contactManager.Destroy(c);
        }
      }

      if (HasFlag(BodyFlags.Enabled)) {
        BroadPhase broadPhase = _world._contactManager.m_broadPhase;
        fixture.DestroyProxies(broadPhase);
      }

      fixture.m_body = null;
      fixture.m_next = null;

      --_fixtureCount;

      // Reset the mass data.
      ResetMassData();
    }

    public void SetTransform(in Vector2 position, float angle) {
      //Debug.Assert(_world.IsLocked() == false);
      if (_world.IsLocked() == true) {
        return;
      }

      _xf.q = Matrix3x2.CreateRotation(angle);//  .Set(angle);
      _xf.p = position;

      _sweep.c = Math.Mul(_xf, _sweep.localCenter);
      _sweep.a = angle;

      _sweep.c0 = _sweep.c;
      _sweep.a0 = angle;

      BroadPhase broadPhase = _world._contactManager.m_broadPhase;
      for (Fixture f = _fixtureList; f != null; f = f.m_next) {
        f.Synchronize(broadPhase, _xf, _xf);
      }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Transform GetTransform() => _xf;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2 GetPosition() => _xf.p;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetAngle() => _sweep.a;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2 GetWorldCenter() => _sweep.c;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2 GetLocalCenter() => _sweep.localCenter;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetLinearVelocity(in Vector2 v) {
      if (_type             == BodyType.Static) return;
      if (Vector2.Dot(v, v) > 0f) SetAwake(true);
      _linearVelocity = v;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2 GetLinearVelocity() => _linearVelocity;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetAngularVelocity(float omega) {
      if (_type         == BodyType.Static) return;
      if (omega * omega > 0f) SetAwake(true);
      _angularVelocity = omega;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetAngularVelocity() => _angularVelocity;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ApplyForce(in Vector2 force, in Vector2 point, bool wake = true) {
      if (_type != BodyType.Dynamic) return;
      if (wake && !HasFlag(BodyFlags.Awake)) SetAwake(true);
      if (HasFlag(BodyFlags.Awake)) {
        _force  += force;
        _torque += Vectex.Cross(point - _sweep.c, force);
      }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ApplyForceToCenter(in Vector2 force, bool wake = true) {
      if (_type != BodyType.Dynamic) return;
      if (wake && !HasFlag(BodyFlags.Awake)) SetAwake(true);
      if (HasFlag(BodyFlags.Awake)) {
        _force += force;
      }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ApplyTorque(float torque, bool wake = true) {
      if (_type != BodyType.Dynamic) return;
      if (wake && !HasFlag(BodyFlags.Awake)) SetAwake(true);
      if (HasFlag(BodyFlags.Awake)) {
        _torque += _torque;
      }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ApplyLinearImpulse(in Vector2 impulse, in Vector2 point, bool wake = true) {
      if (_type != BodyType.Dynamic) return;
      if (wake && !HasFlag(BodyFlags.Awake)) SetAwake(true);
      if (HasFlag(BodyFlags.Awake)) {
        _linearVelocity += _invMass * impulse;
        _torque         += _invI    * Vectex.Cross(point - _sweep.c, impulse);
      }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ApplyLinearImpulseToCenter(in Vector2 impulse, bool wake = true) {
      if (_type != BodyType.Dynamic) return;
      if (wake && !HasFlag(BodyFlags.Awake)) SetAwake(true);
      if (HasFlag(BodyFlags.Awake)) {
        _linearVelocity += _invMass * impulse;
      }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ApplyAngularImpuse(float impulse, bool wake = true) {
      if (_type != BodyType.Dynamic) return;
      if (wake && !HasFlag(BodyFlags.Awake)) SetAwake(true);
      if (HasFlag(BodyFlags.Awake)) {
        _angularVelocity += _invI * impulse;
      }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetMass() => _mass;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetInertia() => _I + _mass * Vector2.Dot(_sweep.localCenter, _sweep.localCenter);

    public void GetMassData(out MassData data) {
      data.mass   = _mass;
      data.I      = _I + _mass * Vector2.Dot(_sweep.localCenter, _sweep.localCenter);
      data.center = _sweep.localCenter;
    }

    public void SetMassData(in MassData massData) {
      //Debug.Assert(_world.IsLocked() == false);
      if (_world.IsLocked() == true) {
        return;
      }

      if (_type != BodyType.Dynamic) {
        return;
      }

      _invMass = 0.0f;
      _I       = 0.0f;
      _invI    = 0.0f;

      _mass = massData.mass;
      if (_mass <= 0.0f) {
        _mass = 1.0f;
      }

      _invMass = 1.0f / _mass;

      if (massData.I > 0.0f && !HasFlag(BodyFlags.FixedRotation)) {
        _I = massData.I - _mass * Vector2.Dot(massData.center, massData.center);
        //Debug.Assert(_I > 0.0f);
        _invI = 1.0f / _I;
      }

      // Move center of mass.
      Vector2 oldCenter = _sweep.c;
      _sweep.localCenter = massData.center;
      _sweep.c0          = _sweep.c = Math.Mul(_xf, _sweep.localCenter);

      // Update center of mass velocity.
      _linearVelocity += Vectex.Cross(_angularVelocity, _sweep.c - oldCenter);
    }

    public void ResetMassData() {
      // Compute mass data from shapes. Each shape has its own density.
      _mass              = 0.0f;
      _invMass           = 0.0f;
      _I                 = 0.0f;
      _invI              = 0.0f;
      _sweep.localCenter = Vector2.Zero;

      // Static and kinematic bodies have zero mass.
      if (_type == BodyType.Static || _type == BodyType.Kinematic) {
        _sweep.c0 = _xf.p;
        _sweep.c  = _xf.p;
        _sweep.a0 = _sweep.a;
        return;
      }

      //Debug.Assert(_type == BodyType.Dynamic);

      // Accumulate mass over all fixtures.
      Vector2 localCenter = Vector2.Zero;
      for (Fixture f = _fixtureList; f != null; f = f.m_next) {
        if (f.m_density == 0.0f) {
          continue;
        }

        f.GetMassData(out MassData massData);
        _mass       += massData.mass;
        localCenter += massData.mass * massData.center;
        _I          += massData.I;
      }

      // Compute center of mass.
      if (_mass > 0.0f) {
        _invMass    =  1.0f / _mass;
        localCenter *= _invMass;
      }

      if (_I > 0.0f && !HasFlag(BodyFlags.FixedRotation)) {
        // Center the inertia about the center of mass.
        _I -= _mass * Vector2.Dot(localCenter, localCenter);
        //Debug.Assert(_I > 0.0f);
        _invI = 1.0f / _I;
      }
      else {
        _I    = 0.0f;
        _invI = 0.0f;
      }

      // Move center of mass.
      Vector2 oldCenter = _sweep.c;
      _sweep.localCenter = localCenter;
      _sweep.c0          = _sweep.c = Math.Mul(_xf, _sweep.localCenter);

      // Update center of mass velocity.
      _linearVelocity += Vectex.Cross(_angularVelocity, _sweep.c - oldCenter);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2 GetWorldPoint(in Vector2 localPoint) => Math.Mul(_xf, localPoint);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2 GetWorldVector(in Vector2 localVector) => Math.Mul(_xf.q, localVector);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2 GetLocalPoint(in Vector2 worldPoint) => Math.MulT(_xf, worldPoint);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2 GetLocalVector(in Vector2 worldVector) => Math.MulT(_xf.q, worldVector);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2 GetLinearVelocityFromWorldPoint(in Vector2 worldPoint) =>
      _linearVelocity + Vectex.Cross(_angularVelocity, worldPoint - _sweep.c);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2 GetLinearVelocityFromLocalPoint(in Vector2 localPoint) =>
      GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetLinearDamping() => _linearDamping;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetLinearDampling(float linearDamping) => _linearDamping = linearDamping;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetAngularDamping() => _angularDamping;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetAngularDamping(float angularDamping) => _angularDamping = angularDamping;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetGravityScale() => _gravityScale;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetGravityScale(float scale) => _gravityScale = scale;

    public void SetType(BodyType type) {
      //Debug.Assert(_world.IsLocked() == false);
      if (_world.IsLocked() == true) {
        return;
      }

      if (_type == type) {
        return;
      }

      _type = type;

      ResetMassData();

      if (_type == BodyType.Static) {
        _linearVelocity  = Vector2.Zero;
        _angularVelocity = 0.0f;
        _sweep.a0        = _sweep.a;
        _sweep.c0        = _sweep.c;
        SynchronizeFixtures();
      }

      SetAwake(true);

      _force  = Vector2.Zero;
      _torque = 0.0f;

      // Delete the attached contacts.
      ContactEdge ce = _contactList;
      while (ce != null) {
        ContactEdge ce0 = ce;
        ce = ce.next;
        _world._contactManager.Destroy(ce0.contact);
      }

      _contactList = null;

      // Touch the proxies so that new contacts will be created (when appropriate)
      BroadPhase broadPhase = _world._contactManager.m_broadPhase;
      for (Fixture f = _fixtureList; f != null; f = f.m_next) {
        int proxyCount = f.m_proxyCount;
        for (int i = 0; i < proxyCount; ++i) {
          broadPhase.TouchProxy(f.m_proxies[i].proxyId);
        }
      }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public BodyType Type() => _type;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void SetFlag(BodyFlags flag) => _flags |= flag;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void UnsetFlag(BodyFlags flag) => _flags &= ~flag;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetBullet(bool flag) {
      if (flag) {
        SetFlag(BodyFlags.Bullet);
      }
      else
        UnsetFlag(BodyFlags.Bullet);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsBullet() => HasFlag(BodyFlags.Bullet);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetSleepingAllowed(bool flag) {
      if (flag) {
        SetFlag(BodyFlags.AutoSleep);
      }
      else {
        UnsetFlag(BodyFlags.AutoSleep);
        SetAwake(true);
      }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsSleepingAllowed() => HasFlag(BodyFlags.AutoSleep);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetAwake(bool flag) {
      if (flag) {
        SetFlag(BodyFlags.Awake);
        _sleepTime = 0f;
      }
      else {
        UnsetFlag(BodyFlags.Awake);
        _sleepTime       = 0f;
        _linearVelocity  = Vector2.Zero;
        _angularVelocity = 0f;
        _force           = Vector2.Zero;
        _torque          = 0f;
      }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsAwake() => HasFlag(BodyFlags.Awake);

    public void SetEnabled(bool flag) {
      //Debug.Assert(_world.IsLocked() == false);

      if (flag == IsEnabled()) {
        return;
      }

      if (flag) {
        SetFlag(BodyFlags.Enabled);

        // Create all proxies.
        BroadPhase broadPhase = _world._contactManager.m_broadPhase;
        for (Fixture f = _fixtureList; f != null; f = f.m_next) {
          f.CreateProxies(broadPhase, _xf);
        }

        // Contacts are created at the beginning of the next
        _world._newContacts = true;
      }
      else {
        UnsetFlag(BodyFlags.Enabled);

        // Destroy all proxies.
        BroadPhase broadPhase = _world._contactManager.m_broadPhase;
        for (Fixture f = _fixtureList; f != null; f = f.m_next) {
          f.DestroyProxies(broadPhase);
        }

        // Destroy the attached contacts.
        ContactEdge ce = _contactList;
        while (ce != null) {
          ContactEdge ce0 = ce;
          ce = ce.next;
          _world._contactManager.Destroy(ce0.contact);
        }

        _contactList = null;
      }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsEnabled() => HasFlag(BodyFlags.Enabled);

    public void SetFixedRotation(bool flag) {
      if (flag == HasFlag(BodyFlags.FixedRotation)) {
        return;
      }

      if (flag) {
        SetFlag(BodyFlags.FixedRotation);
      }
      else {
        UnsetFlag(BodyFlags.FixedRotation);
      }

      _angularVelocity = 0.0f;

      ResetMassData();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsFixedRotation() => HasFlag(BodyFlags.FixedRotation);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixture GetFixtureList() => _fixtureList;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public JointEdge GetJointList() => _jointList;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public ContactEdge GetContactList() => _contactList;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Body GetNext() => _next;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public object GetUserData() => _userData;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetUserData(object data) => _userData = data;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public World.World GetWorld() => _world;

    public void Dump() {
      // Todo: Dump in some form. We could just serialize.
    }

// private

    private Body(in BodyDef bd, World.World world) { }


    internal void SynchronizeFixtures() {
      BroadPhase broadPhase = _world._contactManager.m_broadPhase;

      if (IsAwake()) {
        Transform xf1 = new Transform();
        xf1.q = Matrix3x2.CreateRotation(_sweep.a0);//  .Set(_sweep.a0);
        xf1.p = _sweep.c0 - Math.Mul(xf1.q, _sweep.localCenter);

        for (Fixture f = _fixtureList; f != null; f = f.m_next) {
          f.Synchronize(broadPhase, xf1, _xf);
        }
      }
      else {
        for (Fixture f = _fixtureList; f != null; f = f.m_next) {
          f.Synchronize(broadPhase, _xf, _xf);
        }
      }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void SynchronizeTransform() {
      _xf.q = Matrix3x2.CreateRotation(_sweep.a);// .Set(_sweep.a);
      _xf.p = _sweep.c - Math.Mul(_xf.q, _sweep.localCenter);
    }

    internal bool ShouldCollide(in Body other) {
      // At least one body should be dynamic.
      if (_type != BodyType.Dynamic && other._type != BodyType.Dynamic) {
        return false;
      }

      // Does a joint prevent collision?
      for (JointEdge jn = _jointList; jn != null; jn = jn.next) {
        if (jn.other == other && jn.joint._collideConnected == false)
          return false;
      }

      return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void Advance(float alpha) {
      // Advance to the new safe time. This doesn't sync the broad-phase.
      _sweep.Advance(alpha);
      _sweep.c = _sweep.c0;
      _sweep.a = _sweep.a0;
      _xf.q = Matrix3x2.CreateRotation(_sweep.a); // Set(_sweep.a);
      _xf.p = _sweep.c - Math.Mul(_xf.q, _sweep.localCenter);
    }

    internal BodyType  _type;
    private  BodyFlags _flags;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool HasFlag(BodyFlags flag) => (_flags & flag) == flag;

    internal int _islandIndex;

    internal Transform _xf; // the body origin transform

    internal Sweep _sweep; // the swept motion for CCD

    internal Vector2 _linearVelocity;
    internal float   _angularVelocity;

    internal Vector2 _force;
    internal float   _torque;

    private  World.World _world;
    internal Body  _prev;
    internal Body  _next;

    internal Fixture _fixtureList;
    internal int     _fixtureCount;

    internal JointEdge   _jointList;
    internal ContactEdge _contactList;

    internal float _mass;
    internal float _invMass;
    private  float _I;
    internal float _invI;

    internal float _linearDamping;
    internal float _angularDamping;
    internal float _gravityScale;

    internal float _sleepTime;

    private object _userData;


    internal Body(BodyDef bd, World.World world) {
      //Debug.Assert(bd.position.IsValid());
      //Debug.Assert(bd.linearVelocity.IsValid());

      _flags = 0;

      if (bd.bullet)        SetFlag(BodyFlags.Bullet);
      if (bd.fixedRotation) SetFlag(BodyFlags.FixedRotation);
      if (bd.allowSleep)    SetFlag(BodyFlags.AutoSleep);
      if (bd.awake)         SetFlag(BodyFlags.Awake);
      if (bd.enabled)       SetFlag(BodyFlags.Enabled);

      _world = world;

      _xf.p = bd.position;
      _xf.q = Matrix3x2.CreateRotation(bd.angle); // .Set(bd.angle);

      _sweep.localCenter = Vector2.Zero;
      _sweep.c0          = _xf.p;
      _sweep.c           = _xf.p;
      _sweep.a0          = bd.angle;
      _sweep.a           = bd.angle;
      _sweep.alpha0      = 0.0f;

      _jointList   = null;
      _contactList = null;
      _prev        = null;
      _next        = null;

      _linearVelocity  = bd.linearVelocity;
      _angularVelocity = bd.angularVelocity;

      _linearDamping  = bd.linearDamping;
      _angularDamping = bd.angularDamping;
      _gravityScale   = bd.gravityScale;

      _force  = Vector2.Zero;
      _torque = 0.0f;

      _sleepTime = 0.0f;

      _type = bd.type;

      _mass    = 0.0f;
      _invMass = 0.0f;

      _I    = 0.0f;
      _invI = 0.0f;

      _userData = bd.userData;

      _fixtureList  = null;
      _fixtureCount = 0;
    }


    // This is used to prevent connected bodies from colliding.
    // It may lie, depending on the collideConnected flag.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool IsConnected(Body other) {
      for (JointEdge jn = _jointList; jn != null; jn = jn.next) {
        if (jn.other == other)
          return jn.joint._collideConnected == false;
      }

      return false;
    }
  }
}