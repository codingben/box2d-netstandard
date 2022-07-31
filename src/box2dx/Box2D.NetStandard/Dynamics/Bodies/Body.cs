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

namespace Box2D.NetStandard.Dynamics.Bodies
{
    /// <summary>
    ///  A rigid body. These are created via World.CreateBody.
    /// </summary>
    [DebuggerDisplay("{UserData}")]
    public class Body
    {
        private readonly World.World m_world;
        internal float m_angularDamping;
        internal float m_angularVelocity;
        internal ContactEdge m_contactList;
        internal int m_fixtureCount;

        internal Fixture m_fixtureList;
        private BodyFlags m_flags;

        internal Vector2 m_force;
        internal float m_gravityScale;
        private float m_I;
        internal float m_invI;
        internal float m_invMass;

        internal int m_islandIndex;

        internal JointEdge m_jointList;

        internal float m_linearDamping;

        internal Vector2 m_linearVelocity;

        internal float m_mass;
        internal Body m_next;
        internal Body m_prev;

        internal float m_sleepTime;

        internal Sweep m_sweep; // the swept motion for CCD
        internal float m_torque;

        internal BodyType m_type;

        internal Transform m_xf; // the body origin transform

        // No public default constructor
        private Body()
        { }

        // private

        private Body(in BodyDef bd, World.World world)
        { }


        internal Body(BodyDef bd, World.World world)
        {
            //Debug.Assert(bd.position.IsValid());
            //Debug.Assert(bd.linearVelocity.IsValid());

            m_flags = 0;

            if (bd.bullet)
            {
                SetFlag(BodyFlags.Bullet);
            }

            if (bd.fixedRotation)
            {
                SetFlag(BodyFlags.FixedRotation);
            }

            if (bd.allowSleep)
            {
                SetFlag(BodyFlags.AutoSleep);
            }

            if (bd.awake && bd.type != BodyType.Static)
            {
                SetFlag(BodyFlags.Awake);
            }

            if (bd.enabled)
            {
                SetFlag(BodyFlags.Enabled);
            }

            m_world = world;

            m_xf.p = bd.position;
            m_xf.q = Matrex.CreateRotation(bd.angle); // Actually about twice as fast to use our own function

            m_sweep.localCenter = Vector2.Zero;
            m_sweep.c0 = m_xf.p;
            m_sweep.c = m_xf.p;
            m_sweep.a0 = bd.angle;
            m_sweep.a = bd.angle;
            m_sweep.alpha0 = 0.0f;

            m_jointList = null;
            m_contactList = null;
            m_prev = null;
            m_next = null;

            m_linearVelocity = bd.linearVelocity;
            m_angularVelocity = bd.angularVelocity;

            m_linearDamping = bd.linearDamping;
            m_angularDamping = bd.angularDamping;
            m_gravityScale = bd.gravityScale;

            m_force = Vector2.Zero;
            m_torque = 0.0f;

            m_sleepTime = 0.0f;

            m_type = bd.type;

            m_mass = 0.0f;
            m_invMass = 0.0f;

            m_I = 0.0f;
            m_invI = 0.0f;

            UserData = bd.userData;

            m_fixtureList = null;
            m_fixtureCount = 0;
        }


        public Transform Transform
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => m_xf;
        }

        public Vector2 Position
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => m_xf.p;
        }

        public object UserData
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
            private set;
        }

        // public
        /// <summary>
        ///  Creates a fixture and attaches it to this body. Use this function if you need
        ///  to set some fixture parameters, like friction. Otherwise you can create the
        ///  fixture directly from a shape.
        ///  If the density is non-zero, this function automatically updates the mass of the body.
        ///  Contacts are not created until the next time step.
        /// </summary>
        /// <param name="def">the fixture definition.</param>
        /// <warning>This function is locked during callbacks.</warning>
        public Fixture CreateFixture(in FixtureDef def)
        {
            //Debug.Assert(_world.IsLocked() == false);
            if (m_world.IsLocked())
            {
                throw new
                    Box2DException("Cannot create fixtures in the middle of Step. Has this been spawned from an event such as a ContactListener callback?");
            }

            var fixture = new Fixture();
            fixture.Create(this, def);

            if (HasFlag(BodyFlags.Enabled))
            {
                BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
                fixture.CreateProxies(broadPhase, m_xf);
            }

            fixture.m_next = m_fixtureList;
            m_fixtureList = fixture;
            ++m_fixtureCount;

            fixture.m_body = this;

            // Adjust mass properties if needed.
            if (fixture.m_density > 0.0f)
            {
                ResetMassData();
            }

            // Let the world know we have a new fixture. This will cause new contacts
            // to be created at the beginning of the next time step.
            m_world.m_newContacts = true;

            return fixture;
        }

        /// <summary>
        ///  Creates a fixture from a shape and attach it to this body.
        ///  This is a convenience function. Use b2FixtureDef if you need to set parameters
        ///  like friction, restitution, user data, or filtering.
        ///  If the density is non-zero, this function automatically updates the mass of the body.
        /// </summary>
        /// <param name="shape">the shape to be cloned.</param>
        /// <param name="density">the shape density (set to zero for static bodies).</param>
        /// <warning>This function is locked during callbacks.</warning>
        public Fixture CreateFixture(in Shape shape, float density = 0f)
        {
            var def = new FixtureDef();
            def.shape = shape;
            def.density = density;

            return CreateFixture(def);
        }

        public void DestroyFixture(Fixture fixture)
        {
            if (fixture == null)
            {
                return;
            }

            //Debug.Assert(_world.IsLocked() == false);
            if (m_world.IsLocked())
            {
                throw new
                    Box2DException("Cannot destroy fixtures in the middle of Step. Has this been spawned from an event such as a ContactListener callback?");
            }

            //Debug.Assert(fixture.m_body == this);

            // Remove the fixture from this body's singly linked list.
            //Debug.Assert(_fixtureCount > 0);
            Fixture node = m_fixtureList;
            Fixture prevNode = null;
            bool found = false;
            while (node != null)
            {
                if (node == fixture)
                {
                    if (prevNode == null)
                        m_fixtureList = fixture.m_next;
                    else
                        prevNode.m_next = fixture.m_next;

                    found = true;
                    break;
                }

                prevNode = node;
                node = node.m_next;
            }

            // You tried to remove a shape that is not attached to this body.
            if (!found)
                throw new System.ArgumentException("Fixture does not belong to this Body.", nameof(fixture));

            float density = fixture.m_density;

            // Destroy any contacts associated with the fixture.
            ContactEdge edge = m_contactList;
            while (edge != null)
            {
                Contact c = edge.contact;
                edge = edge.next;

                Fixture fixtureA = c.GetFixtureA();
                Fixture fixtureB = c.GetFixtureB();

                if (fixture == fixtureA || fixture == fixtureB)
                // This destroys the contact and removes it from
                // this body's contact list.
                {
                    m_world.m_contactManager.Destroy(c);
                }
            }

            if (HasFlag(BodyFlags.Enabled))
            {
                BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
                fixture.DestroyProxies(broadPhase);
            }

            fixture.m_body = null;
            fixture.m_next = null;

            --m_fixtureCount;

            if (density > 0.0f)
            {
                // Reset the mass data.
                ResetMassData();
            }
        }

        public void SetTransform(in Vector2 position, float angle)
        {
            //Debug.Assert(_world.IsLocked() == false);
            if (m_world.IsLocked())
            {
                return;
            }

            m_xf.q = Matrex.CreateRotation(angle); //  Actually about twice as fast to use our own function
            m_xf.p = position;

            m_sweep.c = Math.Mul(m_xf, m_sweep.localCenter);
            m_sweep.a = angle;

            m_sweep.c0 = m_sweep.c;
            m_sweep.a0 = angle;

            BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
            for (Fixture f = m_fixtureList; f != null; f = f.m_next)
            {
                f.Synchronize(broadPhase, m_xf, m_xf);
            }

            m_world.m_newContacts = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Transform GetTransform() => m_xf;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2 GetPosition() => m_xf.p;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float GetAngle() => m_sweep.a;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2 GetWorldCenter() => m_sweep.c;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2 GetLocalCenter() => m_sweep.localCenter;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetLinearVelocity(in Vector2 v)
        {
            if (m_type == BodyType.Static)
            {
                return;
            }

            if (Vector2.Dot(v, v) > 0f)
            {
                SetAwake(true);
            }

            m_linearVelocity = v;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2 GetLinearVelocity() => m_linearVelocity;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetAngularVelocity(float omega)
        {
            if (m_type == BodyType.Static)
            {
                return;
            }

            if (omega * omega > 0f)
            {
                SetAwake(true);
            }

            m_angularVelocity = omega;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float GetAngularVelocity() => m_angularVelocity;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyForce(in Vector2 force, in Vector2 point, bool wake = true)
        {
            if (m_type != BodyType.Dynamic)
            {
                return;
            }

            if (wake && !HasFlag(BodyFlags.Awake))
            {
                SetAwake(true);
            }

            if (HasFlag(BodyFlags.Awake))
            {
                m_force += force;
                m_torque += Vectex.Cross(point - m_sweep.c, force);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyForceToCenter(in Vector2 force, bool wake = true)
        {
            if (m_type != BodyType.Dynamic)
            {
                return;
            }

            if (wake && !HasFlag(BodyFlags.Awake))
            {
                SetAwake(true);
            }

            if (HasFlag(BodyFlags.Awake))
            {
                m_force += force;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyTorque(float torque, bool wake = true)
        {
            if (m_type != BodyType.Dynamic)
            {
                return;
            }

            if (wake && !HasFlag(BodyFlags.Awake))
            {
                SetAwake(true);
            }

            if (HasFlag(BodyFlags.Awake))
            {
                m_torque += torque;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyLinearImpulse(in Vector2 impulse, in Vector2 point, bool wake = true)
        {
            if (m_type != BodyType.Dynamic)
            {
                return;
            }

            if (wake && !HasFlag(BodyFlags.Awake))
            {
                SetAwake(true);
            }

            if (HasFlag(BodyFlags.Awake))
            {
                m_linearVelocity += m_invMass * impulse;
                m_torque += m_invI * Vectex.Cross(point - m_sweep.c, impulse);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyLinearImpulseToCenter(in Vector2 impulse, bool wake = true)
        {
            if (m_type != BodyType.Dynamic)
            {
                return;
            }

            if (wake && !HasFlag(BodyFlags.Awake))
            {
                SetAwake(true);
            }

            if (HasFlag(BodyFlags.Awake))
            {
                m_linearVelocity += m_invMass * impulse;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyAngularImpulse(float impulse, bool wake = true)
        {
            if (m_type != BodyType.Dynamic)
            {
                return;
            }

            if (wake && !HasFlag(BodyFlags.Awake))
            {
                SetAwake(true);
            }

            if (HasFlag(BodyFlags.Awake))
            {
                m_angularVelocity += m_invI * impulse;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float GetMass() => m_mass;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float GetInertia() => m_I + m_mass * Vector2.Dot(m_sweep.localCenter, m_sweep.localCenter);

        public void GetMassData(out MassData data)
        {
            data.mass = m_mass;
            data.I = m_I + m_mass * Vector2.Dot(m_sweep.localCenter, m_sweep.localCenter);
            data.center = m_sweep.localCenter;
        }

        public void SetMassData(in MassData massData)
        {
            //Debug.Assert(_world.IsLocked() == false);
            if (m_world.IsLocked())
            {
                return;
            }

            if (m_type != BodyType.Dynamic)
            {
                return;
            }

            m_invMass = 0.0f;
            m_I = 0.0f;
            m_invI = 0.0f;

            m_mass = massData.mass;
            if (m_mass <= 0.0f)
            {
                m_mass = 1.0f;
            }

            m_invMass = 1.0f / m_mass;

            if (massData.I > 0.0f && !HasFlag(BodyFlags.FixedRotation))
            {
                m_I = massData.I - m_mass * Vector2.Dot(massData.center, massData.center);
                //Debug.Assert(_I > 0.0f);
                m_invI = 1.0f / m_I;
            }

            // Move center of mass.
            Vector2 oldCenter = m_sweep.c;
            m_sweep.localCenter = massData.center;
            m_sweep.c0 = m_sweep.c = Math.Mul(m_xf, m_sweep.localCenter);

            // Update center of mass velocity.
            m_linearVelocity += Vectex.Cross(m_angularVelocity, m_sweep.c - oldCenter);
        }

        public void ResetMassData()
        {
            // Compute mass data from shapes. Each shape has its own density.
            m_mass = 0.0f;
            m_invMass = 0.0f;
            m_I = 0.0f;
            m_invI = 0.0f;
            m_sweep.localCenter = Vector2.Zero;

            // Static and kinematic bodies have zero mass.
            if (m_type == BodyType.Static || m_type == BodyType.Kinematic)
            {
                m_sweep.c0 = m_xf.p;
                m_sweep.c = m_xf.p;
                m_sweep.a0 = m_sweep.a;
                return;
            }

            //Debug.Assert(_type == BodyType.Dynamic);

            // Accumulate mass over all fixtures.
            Vector2 localCenter = Vector2.Zero;
            for (Fixture f = m_fixtureList; f != null; f = f.m_next)
            {
                if (f.m_density == 0.0f)
                {
                    continue;
                }

                f.GetMassData(out MassData massData);
                m_mass += massData.mass;
                localCenter += massData.mass * massData.center;
                m_I += massData.I;
            }

            // Compute center of mass.
            if (m_mass > 0.0f)
            {
                m_invMass = 1.0f / m_mass;
                localCenter *= m_invMass;
            }

            if (m_I > 0.0f && !HasFlag(BodyFlags.FixedRotation))
            {
                // Center the inertia about the center of mass.
                m_I -= m_mass * Vector2.Dot(localCenter, localCenter);
                //Debug.Assert(_I > 0.0f);
                m_invI = 1.0f / m_I;
            }
            else
            {
                m_I = 0.0f;
                m_invI = 0.0f;
            }

            // Move center of mass.
            Vector2 oldCenter = m_sweep.c;
            m_sweep.localCenter = localCenter;
            m_sweep.c0 = m_sweep.c = Math.Mul(m_xf, m_sweep.localCenter);

            // Update center of mass velocity.
            m_linearVelocity += Vectex.Cross(m_angularVelocity, m_sweep.c - oldCenter);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2 GetWorldPoint(in Vector2 localPoint) => Math.Mul(m_xf, localPoint);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2 GetWorldVector(in Vector2 localVector) =>
            Vector2.Transform(localVector, m_xf.q); //  Math.Mul(_xf.q, localVector);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2 GetLocalPoint(in Vector2 worldPoint) => Math.MulT(m_xf, worldPoint);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2 GetLocalVector(in Vector2 worldVector) => Math.MulT(m_xf.q, worldVector);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2 GetLinearVelocityFromWorldPoint(in Vector2 worldPoint) =>
            m_linearVelocity + Vectex.Cross(m_angularVelocity, worldPoint - m_sweep.c);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2 GetLinearVelocityFromLocalPoint(in Vector2 localPoint) =>
            GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float GetLinearDamping() => m_linearDamping;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetLinearDampling(float linearDamping)
        {
            m_linearDamping = linearDamping;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float GetAngularDamping() => m_angularDamping;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetAngularDamping(float angularDamping)
        {
            m_angularDamping = angularDamping;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float GetGravityScale() => m_gravityScale;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetGravityScale(float scale)
        {
            m_gravityScale = scale;
        }

        public void SetType(BodyType type)
        {
            //Debug.Assert(_world.IsLocked() == false);
            if (m_world.IsLocked())
            {
                return;
            }

            if (m_type == type)
            {
                return;
            }

            m_type = type;

            ResetMassData();

            if (m_type == BodyType.Static)
            {
                m_linearVelocity = Vector2.Zero;
                m_angularVelocity = 0.0f;
                m_sweep.a0 = m_sweep.a;
                m_sweep.c0 = m_sweep.c;
                m_flags &= ~BodyFlags.Awake;
                SynchronizeFixtures();
            }

            SetAwake(true);

            m_force = Vector2.Zero;
            m_torque = 0.0f;

            // Delete the attached contacts.
            ContactEdge ce = m_contactList;
            while (ce != null)
            {
                ContactEdge ce0 = ce;
                ce = ce.next;
                m_world.m_contactManager.Destroy(ce0.contact);
            }

            m_contactList = null;

            // Touch the proxies so that new contacts will be created (when appropriate)
            BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
            for (Fixture f = m_fixtureList; f != null; f = f.m_next)
            {
                int proxyCount = f.m_proxyCount;
                for (var i = 0; i < proxyCount; ++i)
                {
                    broadPhase.TouchProxy(f.m_proxies[i].proxyId);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BodyType Type() => m_type;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void SetFlag(BodyFlags flag)
        {
            m_flags |= flag;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void UnsetFlag(BodyFlags flag)
        {
            m_flags &= ~flag;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetBullet(bool flag)
        {
            if (flag)
            {
                SetFlag(BodyFlags.Bullet);
            }
            else
            {
                UnsetFlag(BodyFlags.Bullet);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsBullet() => HasFlag(BodyFlags.Bullet);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetSleepingAllowed(bool flag)
        {
            if (flag)
            {
                SetFlag(BodyFlags.AutoSleep);
            }
            else
            {
                UnsetFlag(BodyFlags.AutoSleep);
                SetAwake(true);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsSleepingAllowed() => HasFlag(BodyFlags.AutoSleep);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetAwake(bool flag)
        {
            if (m_type == BodyType.Static)
            {
                return;
            }

            if (flag)
            {
                SetFlag(BodyFlags.Awake);
                m_sleepTime = 0f;
            }
            else
            {
                UnsetFlag(BodyFlags.Awake);
                m_sleepTime = 0f;
                m_linearVelocity = Vector2.Zero;
                m_angularVelocity = 0f;
                m_force = Vector2.Zero;
                m_torque = 0f;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsAwake() => HasFlag(BodyFlags.Awake);

        public void SetEnabled(bool flag)
        {
            //Debug.Assert(_world.IsLocked() == false);

            if (flag == IsEnabled())
            {
                return;
            }

            if (flag)
            {
                SetFlag(BodyFlags.Enabled);

                // Create all proxies.
                BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
                for (Fixture f = m_fixtureList; f != null; f = f.m_next)
                {
                    f.CreateProxies(broadPhase, m_xf);
                }

                // Contacts are created at the beginning of the next
                m_world.m_newContacts = true;
            }
            else
            {
                UnsetFlag(BodyFlags.Enabled);

                // Destroy all proxies.
                BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
                for (Fixture f = m_fixtureList; f != null; f = f.m_next)
                {
                    f.DestroyProxies(broadPhase);
                }

                // Destroy the attached contacts.
                ContactEdge ce = m_contactList;
                while (ce != null)
                {
                    ContactEdge ce0 = ce;
                    ce = ce.next;
                    m_world.m_contactManager.Destroy(ce0.contact);
                }

                m_contactList = null;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsEnabled() => HasFlag(BodyFlags.Enabled);

        public void SetFixedRotation(bool flag)
        {
            if (flag == HasFlag(BodyFlags.FixedRotation))
            {
                return;
            }

            if (flag)
            {
                SetFlag(BodyFlags.FixedRotation);
            }
            else
            {
                UnsetFlag(BodyFlags.FixedRotation);
            }

            m_angularVelocity = 0.0f;

            ResetMassData();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsFixedRotation() => HasFlag(BodyFlags.FixedRotation);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixture GetFixtureList() => m_fixtureList;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public JointEdge GetJointList() => m_jointList;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ContactEdge GetContactList() => m_contactList;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Body GetNext() => m_next;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T GetUserData<T>() => (T)UserData;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetUserData(object data)
        {
            UserData = data;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public World.World GetWorld() => m_world;

        public void Dump()
        {
            // Todo: Dump in some form. We could just serialize.
        }


        internal void SynchronizeFixtures()
        {
            BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;

            if (IsAwake())
            {
                var xf1 = new Transform();
                xf1.q = Matrex.CreateRotation(m_sweep.a0); // Actually about twice as fast to use our own function
                xf1.p = m_sweep.c0 - Vector2.Transform(m_sweep.localCenter, xf1.q); //Math.Mul(xf1.q, _sweep.localCenter);

                for (Fixture f = m_fixtureList; f != null; f = f.m_next)
                {
                    f.Synchronize(broadPhase, xf1, m_xf);
                }
            }
            else
            {
                for (Fixture f = m_fixtureList; f != null; f = f.m_next)
                {
                    f.Synchronize(broadPhase, m_xf, m_xf);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void SynchronizeTransform()
        {
            m_xf.q = Matrex.CreateRotation(m_sweep.a); // Actually about twice as fast to use our own function
            m_xf.p = m_sweep.c - Vector2.Transform(m_sweep.localCenter, m_xf.q); // Math.Mul(_xf.q, _sweep.localCenter);
        }

        internal bool ShouldCollide(in Body other)
        {
            // At least one body should be dynamic.
            if (m_type != BodyType.Dynamic && other.m_type != BodyType.Dynamic)
            {
                return false;
            }

            // Does a joint prevent collision?
            for (JointEdge jn = m_jointList; jn != null; jn = jn.next)
            {
                if (jn.other == other && jn.joint.m_collideConnected == false)
                {
                    return false;
                }
            }

            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void Advance(float alpha)
        {
            // Advance to the new safe time. This doesn't sync the broad-phase.
            m_sweep.Advance(alpha);
            m_sweep.c = m_sweep.c0;
            m_sweep.a = m_sweep.a0;
            m_xf.q = Matrex.CreateRotation(m_sweep.a); // Actually about twice as fast to use our own function
            m_xf.p = m_sweep.c - Vector2.Transform(m_sweep.localCenter, m_xf.q); //Math.Mul(_xf.q, _sweep.localCenter);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool HasFlag(BodyFlags flag) => (m_flags & flag) == flag;


        // This is used to prevent connected bodies from colliding.
        // It may lie, depending on the collideConnected flag.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal bool IsConnected(Body other)
        {
            for (JointEdge jn = m_jointList; jn != null; jn = jn.next)
            {
                if (jn.other == other)
                {
                    return jn.joint.m_collideConnected == false;
                }
            }

            return false;
        }
    }
}