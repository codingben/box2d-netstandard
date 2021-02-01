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
using Box2D.NetStandard.Dynamics.Fixtures;
using Box2D.NetStandard.Dynamics.World.Callbacks;
using Math = Box2D.NetStandard.Common.Math;

namespace Box2D.NetStandard.Dynamics.Contacts
{
	/// <summary>
	///  The class manages contact between two shapes. A contact exists for each overlapping
	///  AABB in the broad-phase (except if filtered). Therefore a contact object may exist
	///  that has no contact points.
	/// </summary>
	public abstract class Contact
	{
		private const byte CircleCircle = (CircleShape.contactMatch << 2) + CircleShape.contactMatch;
		private const byte CircleEdge = (CircleShape.contactMatch << 2) + EdgeShape.contactMatch;
		private const byte CirclePolygon = (CircleShape.contactMatch << 2) + PolygonShape.contactMatch;
		private const byte CircleChain = (CircleShape.contactMatch << 2) + ChainShape.contactMatch;
		private const byte EdgeCircle = (EdgeShape.contactMatch << 2) + CircleShape.contactMatch;
		private const byte EdgeEdge = (EdgeShape.contactMatch << 2) + EdgeShape.contactMatch;
		private const byte EdgePolygon = (EdgeShape.contactMatch << 2) + PolygonShape.contactMatch;
		private const byte EdgeChain = (EdgeShape.contactMatch << 2) + ChainShape.contactMatch;
		private const byte PolygonCircle = (PolygonShape.contactMatch << 2) + CircleShape.contactMatch;
		private const byte PolygonEdge = (PolygonShape.contactMatch << 2) + EdgeShape.contactMatch;
		private const byte PolygonPolygon = (PolygonShape.contactMatch << 2) + PolygonShape.contactMatch;
		private const byte PolygonChain = (PolygonShape.contactMatch << 2) + ChainShape.contactMatch;
		private const byte ChainCircle = (ChainShape.contactMatch << 2) + CircleShape.contactMatch;
		private const byte ChainEdge = (ChainShape.contactMatch << 2) + EdgeShape.contactMatch;
		private const byte ChainPolygon = (ChainShape.contactMatch << 2) + PolygonShape.contactMatch;
		private const byte ChainChain = (ChainShape.contactMatch << 2) + ChainShape.contactMatch;

		internal readonly Fixture m_fixtureA;
		internal readonly Fixture m_fixtureB;

		private readonly int m_indexA;
		private readonly int m_indexB;

		// Nodes for connecting bodies.
		internal readonly ContactEdge m_nodeA;
		internal readonly ContactEdge m_nodeB;

		internal CollisionFlags m_flags;

		internal float m_friction;

		internal Manifold m_manifold = new Manifold();
		internal Contact m_next;

		// World pool and list pointers.
		internal Contact m_prev;
		internal float m_restitution;

		internal float m_tangentSpeed;
		internal float m_toi;

		internal int m_toiCount;

		public Contact(Fixture fA, int indexA, Fixture fB, int indexB)
		{
			m_flags = CollisionFlags.Enabled;

			m_fixtureA = fA;
			m_fixtureB = fB;

			m_indexA = indexA;
			m_indexB = indexB;

			m_manifold.pointCount = 0;

			m_prev = null;
			m_next = null;

			m_nodeA = new ContactEdge();
			m_nodeB = new ContactEdge();

			m_toiCount = 0;

			m_friction = Settings.MixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);
			m_restitution = Settings.MixRestitution(m_fixtureA.Restitution, m_fixtureB.Restitution);

			m_tangentSpeed = 0f;
		}

		internal Manifold Manifold
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => m_manifold;
		}

		public bool Enabled
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => IsEnabled();
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			set => SetEnabled(value);
		}

		public bool Touching
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => IsTouching();
		}

		public Contact Next
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => GetNext();
		}

		/// <summary>
		///  Get the first fixture in this contact.
		/// </summary>
		public Fixture FixtureA
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => m_fixtureA;
		}

		/// <summary>
		///  Get the second fixture in this contact.
		/// </summary>
		public Fixture FixtureB
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => m_fixtureB;
		}

		public float Friction
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => GetFriction();
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			set => SetFriction(value);
		}

		public float Restitution
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => GetRestitution();
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			set => SetRestitution(value);
		}

		public float TangentSpeed
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => GetTangentSpeed();
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			set => SetTangentSpeed(value);
		}

		public int ChildIndexA
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => GetChildIndexA();
		}

		public int ChildIndexB
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => GetChildIndexB();
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal Manifold GetManifold() => m_manifold;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal void GetWorldManifold(out WorldManifold worldManifold)
		{
			Body bodyA = m_fixtureA.Body;
			Body bodyB = m_fixtureB.Body;
			Shape shapeA = m_fixtureA.Shape;
			Shape shapeB = m_fixtureB.Shape;

			worldManifold = new WorldManifold();
			worldManifold.Initialize(m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(),
			                         shapeB.m_radius);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void SetEnabled(bool flag)
		{
			if (flag)
			{
				m_flags |= CollisionFlags.Enabled;
			}
			else
			{
				m_flags &= ~CollisionFlags.Enabled;
			}
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public bool IsEnabled() => (m_flags & CollisionFlags.Enabled) == CollisionFlags.Enabled;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public bool IsTouching() => (m_flags & CollisionFlags.Touching) == CollisionFlags.Touching;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public Contact GetNext() => m_next;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public Fixture GetFixtureA() => FixtureA;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public Fixture GetFixtureB() => FixtureB;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void FlagForFiltering()
		{
			m_flags |= CollisionFlags.Filter;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public float GetFriction() => m_friction;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void SetFriction(float friction)
		{
			m_friction = friction;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void ResetFriction()
		{
			m_friction = Settings.MixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void SetRestitution(float restitution)
		{
			m_restitution = restitution;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public float GetRestitution() => m_restitution;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void ResetRestitution()
		{
			m_restitution = Settings.MixRestitution(m_fixtureA.m_restitution, m_fixtureB.m_restitution);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void SetTangentSpeed(float speed)
		{
			m_tangentSpeed = speed;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public float GetTangentSpeed() => m_tangentSpeed;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public int GetChildIndexA() => m_indexA;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public int GetChildIndexB() => m_indexB;

		internal abstract void Evaluate(out Manifold manifold, in Transform xfA, in Transform xfB);

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Contact Create(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB)
		{
			int match = (fixtureA.Shape.ContactMatch << 2) + fixtureB.Shape.ContactMatch;
			return (match switch
			        {
				        CircleCircle   => new CircleContact(fixtureA, indexA, fixtureB, indexB),
				        CircleEdge     => new EdgeAndCircleContact(fixtureB, indexB, fixtureA, indexA),
				        CirclePolygon  => new PolyAndCircleContact(fixtureB, indexB, fixtureA, indexA),
				        CircleChain    => new ChainAndCircleContact(fixtureB, indexB, fixtureA, indexA),
				        EdgeCircle     => new EdgeAndCircleContact(fixtureA, indexA, fixtureB, indexB),
				        EdgeEdge       => null,
				        EdgePolygon    => new EdgeAndPolygonContact(fixtureA, indexA, fixtureB, indexB),
				        EdgeChain      => null,
				        PolygonCircle  => new PolyAndCircleContact(fixtureA, indexA, fixtureB, indexB),
				        PolygonEdge    => new EdgeAndPolygonContact(fixtureB, indexB, fixtureA, indexA),
				        PolygonPolygon => new PolygonContact(fixtureA, indexA, fixtureB, indexB),
				        PolygonChain   => new ChainAndPolygonContact(fixtureB, indexB, fixtureA, indexA),
				        ChainCircle    => new ChainAndCircleContact(fixtureA, indexA, fixtureB, indexB),
				        ChainEdge      => null,
				        ChainPolygon   => new ChainAndPolygonContact(fixtureA, indexA, fixtureB, indexB),
				        ChainChain     => null,
				        _              => null
			        })!;
		}

		public void Update(ContactListener listener)
		{
			Manifold oldManifold = m_manifold;

			// Re-enable this contact.
			m_flags |= CollisionFlags.Enabled;

			bool touching;
			bool wasTouching = (m_flags & CollisionFlags.Touching) != 0;

			bool sensorA = m_fixtureA.IsSensor();
			bool sensorB = m_fixtureB.IsSensor();
			bool sensor = sensorA || sensorB;

			Body bodyA = m_fixtureA.Body;
			Body bodyB = m_fixtureB.Body;
			Transform xfA = bodyA.Transform;
			Transform xfB = bodyB.Transform;

			// Is this contact a sensor?
			if (sensor)
			{
				Shape shapeA = m_fixtureA.Shape;
				Shape shapeB = m_fixtureB.Shape;
				touching = TestOverlap(shapeA, m_indexA, shapeB, m_indexB, xfA, xfB);

				// Sensors don't generate manifolds.
				m_manifold.pointCount = 0;
			}
			else
			{
				Evaluate(out m_manifold, xfA, xfB);
				touching = m_manifold.pointCount > 0;

				// Match old contact ids to new contact ids and copy the
				// stored impulses to warm start the solver.
				for (var i = 0; i < m_manifold.pointCount; ++i)
				{
					ManifoldPoint mp2 = m_manifold.points[i];
					mp2.normalImpulse = 0.0f;
					mp2.tangentImpulse = 0.0f;
					ContactID id2 = mp2.id;

					for (var j = 0; j < oldManifold.pointCount; ++j)
					{
						ManifoldPoint mp1 = oldManifold.points[j];

						if (mp1.id.key == id2.key)
						{
							mp2.normalImpulse = mp1.normalImpulse;
							mp2.tangentImpulse = mp1.tangentImpulse;
							break;
						}
					}
				}

				if (touching != wasTouching)
				{
					bodyA.SetAwake(true);
					bodyB.SetAwake(true);
				}
			}

			if (touching)
			{
				m_flags |= CollisionFlags.Touching;
			}
			else
			{
				m_flags &= ~CollisionFlags.Touching;
			}

			if (listener != null)
			{
				if (touching && !wasTouching)
				{
					listener.BeginContact(this);
				}

				if (!touching && wasTouching)
				{
					listener.EndContact(this);
				}

				if (touching && !sensor)
				{
					listener.PreSolve(this, oldManifold);
				}
			}
		}

		private bool TestOverlap(in Shape shapeA, int indexA,
			in Shape shapeB, int indexB,
			in Transform xfA, in Transform xfB)
		{
			var input = new DistanceInput();
			input.proxyA.Set(shapeA, indexA);
			input.proxyB.Set(shapeB, indexB);
			input.transformA = xfA;
			input.transformB = xfB;
			input.useRadii = true;

			var cache = new SimplexCache();
			cache.count = 0;

			Distance(out DistanceOutput output, cache, in input);

			return output.distance < 10.0f * Settings.FLT_EPSILON;
		}

		internal static void Distance(out DistanceOutput output, SimplexCache cache, in DistanceInput input)
		{
			output = new DistanceOutput();

			DistanceProxy proxyA = input.proxyA;
			DistanceProxy proxyB = input.proxyB;

			Transform transformA = input.transformA;
			Transform transformB = input.transformB;

			// Initialize the simplex.
			var simplex = new Simplex();
			simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);

			// Get simplex vertices as an array.
			SimplexVertex[] vertices = simplex.m_v;
			const int k_maxIters = 20;

			// These store the vertices of the last simplex so that we
			// can check for duplicates and prevent cycling.
			int[] saveA = new int[3], saveB = new int[3];
			var saveCount = 0;

			// Main iteration loop.
			var iter = 0;
			while (iter < k_maxIters)
			{
				// Copy simplex so we can identify duplicates.
				saveCount = simplex.m_count;
				for (var i = 0; i < saveCount; ++i)
				{
					saveA[i] = vertices[i].indexA;
					saveB[i] = vertices[i].indexB;
				}

				switch (simplex.m_count)
				{
					case 1:
						break;

					case 2:
						simplex.Solve2();
						break;

					case 3:
						simplex.Solve3();
						break;
				}

				// If we have 3 points, then the origin is in the corresponding triangle.
				if (simplex.m_count == 3)
				{
					break;
				}

				// Get search direction.
				Vector2 d = simplex.GetSearchDirection();

				// Ensure the search direction is numerically fit.
				if (d.LengthSquared() < Settings.FLT_EPSILON_SQUARED)
					// The origin is probably contained by a line segment
					// or triangle. Thus the shapes are overlapped.

					// We can't return zero here even though there may be overlap.
					// In case the simplex is a point, segment, or triangle it is difficult
					// to determine if the origin is contained in the CSO or very close to it.
				{
					break;
				}

				// Compute a tentative new simplex vertex using support points.
				ref SimplexVertex vertex = ref vertices[simplex.m_count];
				vertex.indexA = proxyA.GetSupport(Math.MulT(transformA.q, -d));
				vertex.wA = Math.Mul(transformA, proxyA.GetVertex(vertex.indexA));
				vertex.indexB = proxyB.GetSupport(Math.MulT(transformB.q, d));
				vertex.wB = Math.Mul(transformB, proxyB.GetVertex(vertex.indexB));
				vertex.w = vertex.wB - vertex.wA;

				// Iteration count is equated to the number of support point calls.
				++iter;
				//++b2_gjkIters;

				// Check for duplicate support points. This is the main termination criteria.
				var duplicate = false;
				for (var i = 0; i < saveCount; ++i)
				{
					if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i])
					{
						duplicate = true;
						break;
					}
				}

				// If we found a duplicate support point we must exit to avoid cycling.
				if (duplicate)
				{
					break;
				}

				// New vertex is ok and needed.
				++simplex.m_count;
			}

			//_gjkMaxIters = b2Max(b2_gjkMaxIters, iter);

			// Prepare output.
			simplex.GetWitnessPoints(out output.pointA, out output.pointB);
			output.distance = Vector2.Distance(output.pointA, output.pointB);
			output.iterations = iter;

			// Cache the simplex.
			simplex.WriteCache(cache);

			// Apply radii if requested.
			if (input.useRadii)
			{
				float rA = proxyA._radius;
				float rB = proxyB._radius;

				if (output.distance > rA + rB && output.distance > Settings.FLT_EPSILON)
				{
					// Shapes are still no overlapped.
					// Move the witness points to the outer surface.
					output.distance -= rA + rB;
					Vector2 normal = output.pointB - output.pointA;
					normal = Vector2.Normalize(normal);
					output.pointA += rA * normal;
					output.pointB -= rB * normal;
				}
				else
				{
					// Shapes are overlapped when radii are considered.
					// Move the witness points to the middle.
					Vector2 p = 0.5f * (output.pointA + output.pointB);
					output.pointA = p;
					output.pointB = p;
					output.distance = 0.0f;
				}
			}
		}

		public bool ShapeCast(out ShapeCastOutput output, in ShapeCastInput input)
		{
			output.iterations = 0;
			output.lambda = 1.0f;
			output.normal = Vector2.Zero;
			output.point = Vector2.Zero;

			DistanceProxy proxyA = input.proxyA;
			DistanceProxy proxyB = input.proxyB;

			float radiusA = MathF.Max(proxyA._radius, Settings.PolygonRadius);
			float radiusB = MathF.Max(proxyB._radius, Settings.PolygonRadius);
			float radius = radiusA + radiusB;

			Transform xfA = input.transformA;
			Transform xfB = input.transformB;

			Vector2 r = input.translationB;
			Vector2 n = Vector2.Zero;
			var lambda = 0.0f;

			// Initial simplex
			var simplex = new Simplex();
			simplex.m_count = 0;

			// Get simplex vertices as an array.
			SimplexVertex[] vertices = simplex.m_v;

			// Get support point in -r direction
			int indexA = proxyA.GetSupport(Math.MulT(xfA.q, -r));
			Vector2 wA = Math.Mul(xfA, proxyA.GetVertex(indexA));
			int indexB = proxyB.GetSupport(Math.MulT(xfB.q, r));
			Vector2 wB = Math.Mul(xfB, proxyB.GetVertex(indexB));
			Vector2 v = wA - wB;

			// Sigma is the target distance between polygons
			float sigma = MathF.Max(Settings.PolygonRadius, radius - Settings.PolygonRadius);
			const float tolerance = 0.5f * Settings.LinearSlop;

			// Main iteration loop.
			const int k_maxIters = 20;
			var iter = 0;
			while (iter < k_maxIters && MathF.Abs(v.Length() - sigma) > tolerance)
			{
				Debug.Assert(simplex.m_count < 3);

				output.iterations += 1;

				// Support in direction -v (A - B)
				indexA = proxyA.GetSupport(Math.MulT(xfA.q, -v));
				wA = Math.Mul(xfA, proxyA.GetVertex(indexA));
				indexB = proxyB.GetSupport(Math.MulT(xfB.q, v));
				wB = Math.Mul(xfB, proxyB.GetVertex(indexB));
				Vector2 p = wA - wB;

				// -v is a normal at p
				v = Vector2.Normalize(v);

				// Intersect ray with plane
				float vp = Vector2.Dot(v, p);
				float vr = Vector2.Dot(v, r);
				if (vp - sigma > lambda * vr)
				{
					if (vr <= 0.0f)
					{
						return false;
					}

					lambda = (vp - sigma) / vr;
					if (lambda > 1.0f)
					{
						return false;
					}

					n = -v;
					simplex.m_count = 0;
				}

				// Reverse simplex since it works with B - A.
				// Shift by lambda * r because we want the closest point to the current clip point.
				// Note that the support point p is not shifted because we want the plane equation
				// to be formed in unshifted space.
				SimplexVertex vertex = vertices[simplex.m_count];
				vertex.indexA = indexB;
				vertex.wA = wB + lambda * r;
				vertex.indexB = indexA;
				vertex.wB = wA;
				vertex.w = vertex.wB - vertex.wA;
				vertex.a = 1.0f;
				simplex.m_count += 1;

				switch (simplex.m_count)
				{
					case 1:
						break;

					case 2:
						simplex.Solve2();
						break;

					case 3:
						simplex.Solve3();
						break;

					default:
						Debug.Assert(false);
						break;
				}

				// If we have 3 points, then the origin is in the corresponding triangle.
				if (simplex.m_count == 3)
					// Overlap
				{
					return false;
				}

				// Get search direction.
				v = simplex.GetClosestPoint();

				// Iteration count is equated to the number of support point calls.
				++iter;
			}

			// Prepare output.
			simplex.GetWitnessPoints(out Vector2 pointB, out Vector2 pointA);

			if (v.LengthSquared() > 0.0f)
			{
				n = -v;
				n = Vector2.Normalize(n);
			}

			output.point = pointA + radiusA * n;
			output.normal = n;
			output.lambda = lambda;
			output.iterations = iter;
			return true;
		}
	}
}