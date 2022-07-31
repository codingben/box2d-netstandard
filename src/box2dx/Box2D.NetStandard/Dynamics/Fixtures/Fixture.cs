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
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Collision;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Contacts;

namespace Box2D.NetStandard.Dynamics.Fixtures
{
    /// <summary>
    ///  A fixture is used to attach a shape to a body for collision detection. A fixture
    ///  inherits its transform from its parent. Fixtures hold additional non-geometric data
    ///  such as friction, collision filters, etc.
    ///  Fixtures are created via Body.CreateFixture.
    ///  @warning you cannot reuse fixtures.
    /// </summary>
    [DebuggerDisplay("Fixture of {m_body.UserData}")]
    public class Fixture
    {
        internal Body m_body;

        internal float m_density;
        internal Filter m_filter;
        public float m_friction;

        internal Fixture m_next;
        internal FixtureProxy[] m_proxies;
        internal int m_proxyCount;
        internal float m_restitution;

        // non-public default constructor
        internal Fixture()
        {
            UserData = null;
            m_proxyCount = 0;
            m_density = 0f;
        }

        public Shape Shape
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
            private set;
        }

        private bool Sensor
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
            set;
        }

        public Filter FilterData
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => m_filter;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                m_filter = value;
                Refilter();
            }
        }

        public Body Body
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => m_body;
        }

        public Fixture Next
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => m_next;
        }

        public float Density
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => m_density;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set => m_density = value;
        }

        public float Restitution
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => m_restitution;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set => m_restitution = value;
        }

        public object UserData
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
            private set;
        }

        public void Create(Body body, FixtureDef def)
        {
            if (def.shape == null)
            {
                throw new ArgumentNullException("def.shape");
            }

            UserData = def.userData;
            m_friction = def.friction;
            m_restitution = def.restitution;

            m_body = body;
            m_next = null;

            m_filter = def.filter;

            Sensor = def.isSensor;

            Shape = def.shape.Clone();

            // Reserve proxy space
            int childCount = Shape.GetChildCount();
            m_proxies = new FixtureProxy[childCount];
            for (var i = 0; i < childCount; ++i)
            {
                m_proxies[i] = new FixtureProxy();
            }

            m_proxyCount = 0;

            m_density = def.density;
        }

        internal void CreateProxies(BroadPhase broadPhase, in Transform xf)
        {
            //Debug.Assert(m_proxyCount == 0);

            // Create proxies in the broad-phase.
            m_proxyCount = Shape.GetChildCount();

            for (var i = 0; i < m_proxyCount; ++i)
            {
                FixtureProxy proxy = m_proxies[i];
                Shape.ComputeAABB(out proxy.aabb, in xf, i);
                proxy.proxyId = broadPhase.CreateProxy(proxy.aabb, proxy);
                proxy.fixture = this;
                proxy.childIndex = i;
            }
        }

        internal void DestroyProxies(BroadPhase broadPhase)
        {
            // Destroy proxies in the broad-phase.
            for (var i = 0; i < m_proxyCount; ++i)
            {
                FixtureProxy proxy = m_proxies[i];
                broadPhase.DestroyProxy(proxy.proxyId);
                proxy.proxyId = -1;
            }

            m_proxyCount = 0;
        }

        internal void Synchronize(BroadPhase broadPhase, in Transform transform1, in Transform transform2)
        {
            if (m_proxyCount == 0)
            {
                return;
            }

            for (var i = 0; i < m_proxyCount; ++i)
            {
                FixtureProxy proxy = m_proxies[i];

                // Compute an AABB that covers the swept shape (may miss some rotation effect).
                Shape.ComputeAABB(out AABB aabb1, in transform1, proxy.childIndex);
                Shape.ComputeAABB(out AABB aabb2, in transform2, proxy.childIndex);

                proxy.aabb = AABB.Combine(aabb1, aabb2);

                Vector2 displacement = aabb2.GetCenter() - aabb1.GetCenter();

                broadPhase.MoveProxy(proxy.proxyId, proxy.aabb, displacement);
            }
        }

        private void SetFilterData(in Filter filter)
        {
            m_filter = filter;

            Refilter();
        }

        public void Refilter()
        {
            if (m_body == null)
            {
                return;
            }

            // Flag associated contacts for filtering.
            ContactEdge edge = m_body.GetContactList();
            while (edge != null)
            {
                Contact contact = edge.contact;
                Fixture fixtureA = contact.GetFixtureA();
                Fixture fixtureB = contact.GetFixtureB();
                if (fixtureA == this || fixtureB == this)
                {
                    contact.FlagForFiltering();
                }

                edge = edge.next;
            }

            World.World world = m_body.GetWorld();

            if (world == null)
            {
                return;
            }

            // Touch each proxy so that new pairs may be created
            BroadPhase broadPhase = world.m_contactManager.m_broadPhase;
            for (var i = 0; i < m_proxyCount; ++i)
            {
                broadPhase.TouchProxy(m_proxies[i].proxyId);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsSensor() => Sensor;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private Filter GetFilterData() => m_filter;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Body GetBody() => Body;

        public Fixture GetNext() => m_next;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TestPoint(in Vector2 p) => Shape.TestPoint(m_body.GetTransform(), p);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RayCast(out RayCastOutput output, in RayCastInput input, int childIndex) =>
            Shape.RayCast(out output, in input, m_body.GetTransform(), childIndex);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetMassData(out MassData massData)
        {
            Shape.ComputeMass(out massData, m_density);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public AABB GetAABB(int childIndex) => m_proxies[childIndex].aabb;
    }
}