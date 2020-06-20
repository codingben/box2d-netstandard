/*
  Box2DX Copyright (c) 2009 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com

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

using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Collision;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Contacts;
using int32 = System.Int32;
using b2Vec2 = System.Numerics.Vector2; 

namespace Box2D.NetStandard.Dynamics.Fixture
{
	/// <summary>
	/// A fixture is used to attach a shape to a body for collision detection. A fixture
	/// inherits its transform from its parent. Fixtures hold additional non-geometric data
	/// such as friction, collision filters, etc.
	/// Fixtures are created via Body.CreateFixture.
	/// @warning you cannot reuse fixtures.
	/// </summary>
	[DebuggerDisplay("Fixture of {m_body._userData}")]
	public class Fixture
	{
		internal float m_density;

		internal Fixture m_next;
		internal Body.Body m_body;

		private Shape m_shape;

		internal float m_friction;
		internal float m_restitution;

		internal FixtureProxy[] m_proxies;
		internal int            m_proxyCount;

		internal Filter m_filter;

		private bool m_isSensor;

		private object m_userData;


		internal Fixture()
		{
			m_userData = null;
			m_body = null;
			m_next = null;
			m_proxies = null;
			m_proxyCount = 0;
			m_shape = null;
			m_density = 0f;
		}

		public void Create(Body.Body body, FixtureDef def) {
			if (def.shape ==null) throw new ArgumentNullException("def.shape");

			m_userData    = def.userData;
			m_friction    = def.friction;
			m_restitution = def.restitution;

			m_body = body;
			m_next = null;

			m_filter = def.Filter;

			m_isSensor = def.isSensor;

			m_shape = def.shape.Clone();

			// Reserve proxy space
			int childCount = m_shape.GetChildCount();
			m_proxies = new FixtureProxy[childCount];
			for (int i = 0; i < childCount; ++i) {
				m_proxies[i] = new FixtureProxy();
			}
			m_proxyCount = 0;

			m_density = def.density;
		}

		public void Destroy()
		{
// The proxies must be destroyed before calling this.
			Debug.Assert(m_proxyCount == 0);

			// Free the proxy array.
			int32 childCount = m_shape.GetChildCount();
			m_proxies = null;
			
			m_shape = null;
		}

		internal void CreateProxies(BroadPhase broadPhase, in Transform xf)
		{
			Debug.Assert(m_proxyCount == 0);

			// Create proxies in the broad-phase.
			m_proxyCount = m_shape.GetChildCount();

			for (int32 i = 0; i < m_proxyCount; ++i)
			{
				
				FixtureProxy proxy = m_proxies[i];
				m_shape.ComputeAABB(out proxy.aabb, in xf, i);
				proxy.proxyId    = broadPhase.CreateProxy(proxy.aabb, proxy);
				proxy.fixture    = this;
				proxy.childIndex = i;
			}
		}

		internal void DestroyProxies(BroadPhase broadPhase)
		{
			// Destroy proxies in the broad-phase.
			for (int32 i = 0; i < m_proxyCount; ++i)
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

			for (int32 i = 0; i < m_proxyCount; ++i)
			{
				FixtureProxy proxy = m_proxies[i];

				// Compute an AABB that covers the swept shape (may miss some rotation effect).
				m_shape.ComputeAABB(out AABB aabb1, in transform1, proxy.childIndex);
				m_shape.ComputeAABB(out AABB aabb2, in transform2, proxy.childIndex);
	
				proxy.aabb.Combine(aabb1, aabb2);

				b2Vec2 displacement = aabb2.GetCenter() - aabb1.GetCenter();

				broadPhase.MoveProxy(proxy.proxyId, proxy.aabb, displacement);
			}
		}

		void SetFilterData(in Filter filter)
		{
			m_filter = filter;

			Refilter();
		}
		
		void Refilter()
		{
			if (m_body == null)
			{
				return;
			}

			// Flag associated contacts for filtering.
			ContactEdge edge = m_body.GetContactList();
			while (edge!=null)
			{
				Contact contact  = edge.contact;
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
			BroadPhase broadPhase = world._contactManager.m_broadPhase;
			for (int32 i = 0; i < m_proxyCount; ++i)
			{
				broadPhase.TouchProxy(m_proxies[i].proxyId);
			}
		}

		public ShapeType Type {
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get=>m_shape.m_type;
		}

		public Shape Shape {
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => m_shape;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public bool IsSensor() => m_isSensor;

		private bool Sensor {
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => m_isSensor;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]

		Filter GetFilterData() => m_filter;

		private Filter FilterData {
			[MethodImpl(MethodImplOptions.AggressiveInlining)]

			get => m_filter;
		}

		public Body.Body Body {
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => m_body;
		}
		
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public Body.Body GetBody() => Body;

		Fixture GetNext() => m_next;
		public Fixture Next {
			[MethodImpl(MethodImplOptions.AggressiveInlining)]

			get => m_next;
		}

		public float Density {
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => m_density;
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			set => m_density = value;
		}

		public float Restitution {
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => m_restitution;
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			set => m_restitution = value;
		}
		
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public bool TestPoint(in Vector2 p) => m_shape.TestPoint(m_body.GetTransform(), p);

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public bool RayCast(out RayCastOutput output, in RayCastInput input, int childIndex) =>
			m_shape.RayCast(out output, in input, m_body.GetTransform(), childIndex);

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void GetMassData(out MassData massData) => m_shape.ComputeMass(out massData, m_density);

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public AABB GetAABB(int childIndex) => m_proxies[childIndex].aabb;

	}
	
	
	
}
