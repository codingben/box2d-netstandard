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
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Collision;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Contacts;
using Box2D.NetStandard.Dynamics.Fixtures;
using Box2D.NetStandard.Dynamics.Joints;
using Box2D.NetStandard.Dynamics.Joints.Distance;
using Box2D.NetStandard.Dynamics.Joints.Mouse;
using Box2D.NetStandard.Dynamics.Joints.Pulley;
using Box2D.NetStandard.Dynamics.World.Callbacks;
using Math = Box2D.NetStandard.Common.Math;

namespace Box2D.NetStandard.Dynamics.World
{
	/// <summary>
	///  The world class manages all physics entities, dynamic simulation,
	///  and asynchronous queries.
	/// </summary>
	public class World
	{
		public delegate bool QueryCallback(Fixture fixture);

		public delegate void RayCastCallback(Fixture fixture, Vector2 point, Vector2 normal, float fraction);

		private readonly bool m_clearForces;

		//internal BroadPhase     _broadPhase;
		internal readonly ContactManager m_contactManager;
		private readonly bool m_continuousPhysics;
		private readonly bool m_subStepping;

		// These are for debugging the solver
		private readonly bool m_warmStarting;

		private Action DrawDebugDataStub = () => { };
		private bool m_allowSleep;

		private int m_bodyCount;

		private Body m_bodyList;
		private DebugDraw m_debugDraw;

		private DestructionListener m_destructionListener;

		private Vector2 m_gravity;

		private float m_inv_dt0;
		private int m_jointCount;
		private Joint m_jointList;
		private bool m_locked;

		internal bool m_newContacts;

		private bool m_stepComplete;

		public World() : this(new Vector2(0, -10))
		{ }

		/// <summary>
		///  Construct a world object.
		/// </summary>
		/// <param name="worldAABB">A bounding box that completely encompasses all your shapes.</param>
		/// <param name="gravity">The world gravity vector.</param>
		/// <param name="doSleep">Improve performance by not simulating inactive bodies.</param>
		public World(Vector2 gravity)
		{
			m_destructionListener = null;
			m_debugDraw = null;

			m_bodyList = null;
			m_jointList = null;

			m_bodyCount = 0;
			m_jointCount = 0;

			m_warmStarting = true;
			m_continuousPhysics = true;
			m_subStepping = false;

			m_stepComplete = true;
			m_allowSleep = true;
			m_gravity = gravity;

			m_newContacts = false;
			m_locked = false;
			m_clearForces = true;

			m_inv_dt0 = 0.0f;

			m_contactManager = new ContactManager();
		}

		/// <summary>
		///  Get\Set global gravity vector.
		/// </summary>
		public Vector2 Gravity
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => m_gravity;
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			set => m_gravity = value;
		}

		/// <summary>
		///  Get the world body list. With the returned body, use Body.GetNext to get
		///  the next body in the world list. A null body indicates the end of the list.
		/// </summary>
		/// <returns>The head of the world body list.</returns>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public Body GetBodyList() => m_bodyList;

		/// <summary>
		///  Get the world joint list. With the returned joint, use Joint.GetNext to get
		///  the next joint in the world list. A null joint indicates the end of the list.
		/// </summary>
		/// <returns>The head of the world joint list.</returns>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public Joint GetJointList() => m_jointList;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public Contact GetContactList() => m_contactManager.m_contactList;

		/// <summary>
		///  Get the number of bodies.
		/// </summary>
		/// <returns></returns>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public int GetBodyCount() => m_bodyCount;

		/// <summary>
		///  Get the number joints.
		/// </summary>
		/// <returns></returns>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public int GetJointCount() => m_jointCount;

		/// <summary>
		///  Get the number of contacts (each may have 0 or more contact points).
		/// </summary>
		/// <returns></returns>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public int GetContactCount() => m_contactManager.m_contactCount;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void SetGravity(in Vector2 gravity)
		{
			m_gravity = gravity;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public Vector2 GetGravity() => m_gravity;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public bool IsLocked() => m_locked;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public bool GetAutoClearGorces() => m_clearForces;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal ContactManager GetContactManager() => m_contactManager;

		/// <summary>
		///  Register a destruction listener.
		/// </summary>
		/// <param name="listener"></param>
		public void SetDestructionListener(DestructionListener listener)
		{
			m_destructionListener = listener;
		}

		/// <summary>
		///  Register a contact filter to provide specific control over collision.
		///  Otherwise the default filter is used (b2_defaultFilter).
		/// </summary>
		/// <param name="filter"></param>
		public void SetContactFilter(ContactFilter filter)
		{
			m_contactManager.m_contactFilter = filter;
		}

		/// <summary>
		///  Register a contact event listener
		/// </summary>
		/// <param name="listener"></param>
		public void SetContactListener(ContactListener listener)
		{
			m_contactManager.m_contactListener = listener;
		}

		/// <summary>
		///  Register a routine for debug drawing. The debug draw functions are called
		///  inside the World.Step method, so make sure your renderer is ready to
		///  consume draw commands when you call Step().
		/// </summary>
		/// <param name="debugDraw"></param>
		public void SetDebugDraw(DebugDraw debugDraw)
		{
			m_debugDraw = debugDraw;
			DrawDebugDataStub = DrawDebugData;
		}

		/// <summary>
		///  Create a rigid body given a definition. No reference to the definition
		///  is retained.
		///  @warning This function is locked during callbacks.
		/// </summary>
		/// <param name="def"></param>
		/// <returns></returns>
		public Body CreateBody(BodyDef def)
		{
			//Debug.Assert(_locked == false);

			if (m_locked)
			{
				throw new
					Box2DException("Cannot create bodies in the middle of Step. Has this been spawned from an event such as a ContactListener callback?");
			}


			var b = new Body(def, this);

			// Add to world doubly linked list.
			b.m_prev = null;
			b.m_next = m_bodyList;
			if (m_bodyList != null)
			{
				m_bodyList.m_prev = b;
			}

			m_bodyList = b;
			++m_bodyCount;

			return b;
		}

		/// <summary>
		///  Destroy a rigid body given a definition. No reference to the definition
		///  is retained. This function is locked during callbacks.
		///  @warning This automatically deletes all associated shapes and joints.
		///  @warning This function is locked during callbacks.
		/// </summary>
		/// <param name="b"></param>
		public void DestroyBody(Body b)
		{
			//Debug.Assert(_bodyCount > 0);
			//Debug.Assert(_locked    == false);
			if (m_locked)
			{
				throw new
					Box2DException("Cannot destroy bodies in the middle of Step. Has this been spawned from an event such as a ContactListener callback?");
			}
			
			// Delete the attached joints.
			JointEdge je = b.m_jointList;
			while (je != null)
			{
				JointEdge je0 = je;
				je = je.next;

				m_destructionListener?.SayGoodbye(je0.joint);

				DestroyJoint(je0.joint);

				b.m_jointList = je;
			}

			b.m_jointList = null;

			ContactEdge ce = b.m_contactList;
			while (ce != null)
			{
				ContactEdge ce0 = ce;
				ce = ce.next;
				m_contactManager.Destroy(ce0.contact);
			}

			b.m_contactList = null;

			// Delete the attached fixtures. This destroys broad-phase
			// proxies.
			Fixture f = b.m_fixtureList;
			while (f != null)
			{
				Fixture f0 = f;
				f = f.m_next;

				m_destructionListener?.SayGoodbye(f0);

				f0.DestroyProxies(m_contactManager.m_broadPhase);

				b.m_fixtureList = f;
				b.m_fixtureCount -= 1;
			}

			b.m_fixtureList = null;
			b.m_fixtureCount = 0;

			// Remove world body list.
			if (b.m_prev != null)
			{
				b.m_prev.m_next = b.m_next;
			}

			if (b.m_next != null)
			{
				b.m_next.m_prev = b.m_prev;
			}

			if (b == m_bodyList)
			{
				m_bodyList = b.m_next;
			}

			--m_bodyCount;
			b = null;
		}

		/// <summary>
		///  Create a joint to constrain bodies together. No reference to the definition
		///  is retained. This may cause the connected bodies to cease colliding.
		///  @warning This function is locked during callbacks.
		/// </summary>
		/// <param name="def"></param>
		/// <returns></returns>
		public Joint CreateJoint(JointDef def)
		{
			//Debug.Assert(_locked == false);

			if (m_locked)
			{
				throw new
					Box2DException("Cannot create joints in the middle of Step. Has this been spawned from an event such as a ContactListener callback?");
			}

			var j = Joint.Create(def);

			// Connect to the world list.
			j.m_prev = null;
			j.m_next = m_jointList;
			if (m_jointList != null)
			{
				m_jointList.m_prev = j;
			}

			m_jointList = j;
			++m_jointCount;

			// Connect to the bodies' doubly linked lists.
			j.m_edgeA.joint = j;
			j.m_edgeA.other = j.m_bodyB;
			j.m_edgeA.Prev = null;
			j.m_edgeA.next = j.m_bodyA.m_jointList;
			if (j.m_bodyA.m_jointList != null)
			{
				j.m_bodyA.m_jointList.Prev = j.m_edgeA;
			}

			j.m_bodyA.m_jointList = j.m_edgeA;

			j.m_edgeB.joint = j;
			j.m_edgeB.other = j.m_bodyA;
			j.m_edgeB.Prev = null;
			j.m_edgeB.next = j.m_bodyB.m_jointList;
			if (j.m_bodyB.m_jointList != null)
			{
				j.m_bodyB.m_jointList.Prev = j.m_edgeB;
			}

			j.m_bodyB.m_jointList = j.m_edgeB;

			Body bodyA = def.bodyA;
			Body bodyB = def.bodyB;

			// If the joint prevents collisions, then flag any contacts for filtering.
			if (def.collideConnected == false)
			{
				ContactEdge edge = bodyB.m_contactList;
				while (edge != null)
				{
					if (edge.other == bodyA)
						// Flag the contact for filtering at the next time step (where either
						// body is awake).
					{
						edge.contact.FlagForFiltering();
					}

					edge = edge.next;
				}
			}

			// Note: creating a joint doesn't wake the bodies.

			return j;
		}

		/// <summary>
		///  Destroy a joint. This may cause the connected bodies to begin colliding.
		///  @warning This function is locked during callbacks.
		/// </summary>
		/// <param name="j"></param>
		public void DestroyJoint(Joint j)
		{
			//Debug.Assert(_locked == false);
			if (m_locked)
			{
				throw new
					Box2DException("Cannot destroy joints in the middle of Step. Has this been spawned from an event such as a ContactListener callback?");
			}

			bool collideConnected = j.m_collideConnected;

			// Remove from the doubly linked list.
			if (j.m_prev != null)
			{
				j.m_prev.m_next = j.m_next;
			}

			if (j.m_next != null)
			{
				j.m_next.m_prev = j.m_prev;
			}

			if (j == m_jointList)
			{
				m_jointList = j.m_next;
			}

			// Disconnect from island graph.
			Body bodyA = j.m_bodyA;
			Body bodyB = j.m_bodyB;

			// Wake up connected bodies.
			bodyA.SetAwake(true);
			bodyB.SetAwake(true);

			// Remove from body 1.
			if (j.m_edgeA.Prev != null)
			{
				j.m_edgeA.Prev.next = j.m_edgeA.next;
			}

			if (j.m_edgeA.next != null)
			{
				j.m_edgeA.next.Prev = j.m_edgeA.Prev;
			}

			if (j.m_edgeA == bodyA.m_jointList)
			{
				bodyA.m_jointList = j.m_edgeA.next;
			}

			j.m_edgeA.Prev = null;
			j.m_edgeA.next = null;

			// Remove from body 2
			if (j.m_edgeB.Prev != null)
			{
				j.m_edgeB.Prev.next = j.m_edgeB.next;
			}

			if (j.m_edgeB.next != null)
			{
				j.m_edgeB.next.Prev = j.m_edgeB.Prev;
			}

			if (j.m_edgeB == bodyB.m_jointList)
			{
				bodyB.m_jointList = j.m_edgeB.next;
			}

			j.m_edgeB.Prev = null;
			j.m_edgeB.next = null;

			//Debug.Assert(_jointCount > 0);
			--m_jointCount;

			// If the joint prevents collisions, then flag any contacts for filtering.
			if (collideConnected == false)
			{
				ContactEdge edge = bodyB.m_contactList;
				while (edge != null)
				{
					if (edge.other == bodyA)
						// Flag the contact for filtering at the next time step (where either
						// body is awake).
					{
						edge.contact.FlagForFiltering();
					}

					edge = edge.next;
				}
			}
		}

		public void SetAllowSleeping(bool flag)
		{
			if (flag == m_allowSleep)
			{
				return;
			}

			m_allowSleep = flag;
			if (!m_allowSleep)
			{
				for (Body b = m_bodyList; b != null; b = b.m_next)
				{
					b.SetAwake(true);
				}
			}
		}

		// Find islands, integrate and solve constraints, solve position constraints
		private void Solve(TimeStep step)
		{
			// Size the island for the worst case.
			var island = new Island(m_bodyCount,
			                        m_contactManager.m_contactCount,
			                        m_jointCount,
			                        m_contactManager.m_contactListener);

			// Clear all the island flags.
			for (Body b = m_bodyList; b != null; b = b.m_next)
			{
				b.UnsetFlag(BodyFlags.Island);
			}

			for (Contact c = m_contactManager.m_contactList; c != null; c = c.m_next)
			{
				c.m_flags &= ~CollisionFlags.Island;
			}

			for (Joint j = m_jointList; j != null; j = j.m_next)
			{
				j.m_islandFlag = false;
			}

			// Build and simulate all awake islands.
			int stackSize = m_bodyCount;
			//Stack<Body> stack = new Stack<Body>(_bodyCount);
			//Body stack     = (b2Body**) m_stackAllocator.Allocate(stackSize * sizeof(b2Body*));
			var stack = new Body[m_bodyCount];
			for (Body seed = m_bodyList; seed != null; seed = seed.m_next)
			{
				if (seed.HasFlag(BodyFlags.Island))
				{
					continue;
				}

				if (seed.IsAwake() == false || seed.IsEnabled() == false)
				{
					continue;
				}

				// The seed can be dynamic or kinematic.
				if (seed.m_type == BodyType.Static)
				{
					continue;
				}

				// Reset island and stack.
				island.Clear();
				var stackCount = 0;
				stack[stackCount++] = seed;
				seed.SetFlag(BodyFlags.Island);

				// Perform a depth first search (DFS) on the constraint graph.
				while (stackCount > 0)
				{
					// Grab the next body off the stack and add it to the island.
					Body b = stack[--stackCount];
					////Debug.Assert(b.IsEnabled() == true);
					island.Add(b);

					// To keep islands as small as possible, we don't
					// propagate islands across static bodies.
					if (b.m_type == BodyType.Static)
					{
						continue;
					}

					// Make sure the body is awake (without resetting sleep timer).
					b.SetFlag(BodyFlags.Awake);

					// Search all contacts connected to this body.
					for (ContactEdge ce = b.m_contactList; ce != null; ce = ce.next)
					{
						Contact contact = ce.contact;

						// Has this contact already been added to an island?
						if ((contact.m_flags & CollisionFlags.Island) == CollisionFlags.Island)
						{
							continue;
						}

						// Is this contact solid and touching?
						if (contact.IsEnabled() == false ||
						    contact.IsTouching() == false)
						{
							continue;
						}

						// Skip sensors.
						bool sensorA = contact.m_fixtureA.IsSensor();
						bool sensorB = contact.m_fixtureB.IsSensor();
						if (sensorA || sensorB)
						{
							continue;
						}

						island.Add(contact);
						contact.m_flags |= CollisionFlags.Island;

						Body other = ce.other;

						// Was the other body already added to this island?
						if (other.HasFlag(BodyFlags.Island))
						{
							continue;
						}

						//Debug.Assert(stackCount < stackSize);
						stack[stackCount++] = other;
						other.SetFlag(BodyFlags.Island);
					}

					// Search all joints connect to this body.
					for (JointEdge je = b.m_jointList; je != null; je = je.next)
					{
						if (je.joint.m_islandFlag)
						{
							continue;
						}

						Body other = je.other;

						// Don't simulate joints connected to disabled bodies.
						if (other.IsEnabled() == false)
						{
							continue;
						}

						island.Add(je.joint);
						je.joint.m_islandFlag = true;

						if (other.HasFlag(BodyFlags.Island))
						{
							continue;
						}

						//Debug.Assert(stackCount < stackSize);
						stack[stackCount++] = other;
						other.SetFlag(BodyFlags.Island);
					}
				}

				island.Solve(step, m_gravity, m_allowSleep);

				// Post solve cleanup.
				for (var i = 0; i < island.m_bodyCount; ++i)
				{
					// Allow static bodies to participate in other islands.
					Body b = island.m_bodies[i];
					if (b.m_type == BodyType.Static)
					{
						b.UnsetFlag(BodyFlags.Island);
					}
				}
			}

			stack = null;

			{
				// Synchronize fixtures, check for out of range bodies.
				for (Body b = m_bodyList; b != null; b = b.GetNext())
				{
					// If a body was not in an island then it did not move.
					if (!b.HasFlag(BodyFlags.Island))
					{
						continue;
					}

					if (b.m_type == BodyType.Static)
					{
						continue;
					}

					// Update fixtures (for broad-phase).
					b.SynchronizeFixtures();
				}

				// Look for new contacts.
				m_contactManager.FindNewContacts();
			}
		}

		// Find TOI contacts and solve them.
		private void SolveTOI(in TimeStep step)
		{
			var island = new Island(2 * Settings.MaxTOIContacts, Settings.MaxTOIContacts, 0,
			                        m_contactManager.m_contactListener);

			if (m_stepComplete)
			{
				for (Body b = m_bodyList; b != null; b = b.m_next)
				{
					b.UnsetFlag(BodyFlags.Island);
					b.m_sweep.alpha0 = 0.0f;
				}

				for (Contact c = m_contactManager.m_contactList; c != null; c = c.m_next)
				{
					// Invalidate TOI
					c.m_flags &= ~(CollisionFlags.Toi | CollisionFlags.Island);
					c.m_toiCount = 0;
					c.m_toi = 1.0f;
				}
			}

			// Find TOI events and solve them.
			for (;;)
			{
				// Find the first TOI.
				Contact minContact = null;
				var minAlpha = 1.0f;

				for (Contact c = m_contactManager.m_contactList; c != null; c = c.m_next)
				{
					// Is this contact disabled?
					if (c.Enabled == false)
					{
						continue;
					}

					// Prevent excessive sub-stepping.
					if (c.m_toiCount > Settings.MaxSubSteps)
					{
						continue;
					}

					var alpha = 1.0f;
					if ((c.m_flags & CollisionFlags.Toi) == CollisionFlags.Toi)
					{
						// This contact has a valid cached TOI.
						alpha = c.m_toi;
					}
					else
					{
						Fixture fA = c.FixtureA;
						Fixture fB = c.FixtureB;

						// Is there a sensor?
						if (fA.IsSensor() || fB.IsSensor())
						{
							continue;
						}

						Body bA = fA.Body;
						Body bB = fB.Body;

						bool activeA = bA.IsAwake() && bA.m_type != BodyType.Static;
						bool activeB = bB.IsAwake() && bB.m_type != BodyType.Static;

						// Is at least one body active (awake and dynamic or kinematic)?
						if (activeA == false && activeB == false)
						{
							continue;
						}

						bool collideA = bA.IsBullet() || bA.m_type != BodyType.Dynamic;
						bool collideB = bB.IsBullet() || bB.m_type != BodyType.Dynamic;

						// Are these two non-bullet dynamic bodies?
						if (collideA == false && collideB == false)
						{
							continue;
						}

						// Compute the TOI for this contact.
						// Put the sweeps onto the same time interval.
						float alpha0 = bA.m_sweep.alpha0;

						if (bA.m_sweep.alpha0 < bB.m_sweep.alpha0)
						{
							bA.m_sweep.Advance(alpha0 = bB.m_sweep.alpha0);
						}
						else if (bB.m_sweep.alpha0 < bA.m_sweep.alpha0)
						{
							bB.m_sweep.Advance(alpha0);
						}

						//Debug.Assert(alpha0 < 1.0f);

						// Compute the time of impact in interval [0, minTOI]
						TOIInput input;
						input.proxyA = new DistanceProxy();
						input.proxyA.Set(fA.Shape, c.ChildIndexA);
						input.proxyB = new DistanceProxy();
						input.proxyB.Set(fB.Shape, c.ChildIndexB);
						input.sweepA = bA.m_sweep;
						input.sweepB = bB.m_sweep;
						input.tMax = 1.0f;

						TOI.TimeOfImpact(out TOIOutput output, in input);

						// Beta is the fraction of the remaining portion of the .
						if (output.state == TOIOutputState.Touching)
						{
							alpha = MathF.Min(alpha0 + (1.0f - alpha0) * output.t, 1.0f);
						}
						else
						{
							alpha = 1.0f;
						}

						c.m_toi = alpha;
						c.m_flags |= CollisionFlags.Toi;
					}

					if (alpha < minAlpha)
					{
						// This is the minimum TOI found so far.
						minContact = c;
						minAlpha = alpha;
					}
				}

				if (minContact == null || 1.0f - 10.0f * Settings.FLT_EPSILON < minAlpha)
				{
					// No more TOI events. Done!
					m_stepComplete = true;
					break;
				}

				{
					// Advance the bodies to the TOI.
					Fixture fA = minContact.m_fixtureA;
					Fixture fB = minContact.m_fixtureB;
					Body bA = fA.Body;
					Body bB = fB.Body;

					Sweep backup1 = bA.m_sweep;
					Sweep backup2 = bB.m_sweep;

					bA.Advance(minAlpha);
					bB.Advance(minAlpha);

					// The TOI contact likely has some new contact points.
					minContact.Update(m_contactManager.m_contactListener);
					minContact.m_flags &= ~CollisionFlags.Toi;
					++minContact.m_toiCount;

					// Is the contact solid?
					if (minContact.IsEnabled() == false || minContact.IsTouching() == false)
					{
						// Restore the sweeps.
						minContact.SetEnabled(false);
						bA.m_sweep = backup1;
						bB.m_sweep = backup2;
						bA.SynchronizeTransform();
						bB.SynchronizeTransform();
						continue;
					}

					bA.SetAwake(true);
					bB.SetAwake(true);

					// Build the island
					island.Clear();
					island.Add(bA);
					island.Add(bB);
					island.Add(minContact);

					bA.SetFlag(BodyFlags.Island);
					bB.SetFlag(BodyFlags.Island);
					minContact.m_flags |= CollisionFlags.Island;

					// Get contacts on bodyA and bodyB.
					Body[] bodies = {bA, bB};

					for (var i = 0; i < 2; ++i)
					{
						Body body = bodies[i];
						if (body.m_type == BodyType.Dynamic)
						{
							for (ContactEdge ce = body.m_contactList; ce != null; ce = ce.next)
							{
								if (island.m_bodyCount == island.m_bodyCapacity)
								{
									break;
								}

								if (island.m_contactCount == island.m_contactCapacity)
								{
									break;
								}

								Contact contact = ce.contact;

								// Has this contact already been added to the island?
								if ((contact.m_flags & CollisionFlags.Island) == CollisionFlags.Island)
								{
									continue;
								}

								// Only add static, kinematic, or bullet bodies.
								Body other = ce.other;
								if (other.m_type == BodyType.Dynamic &&
								    body.IsBullet() == false && other.IsBullet() == false)
								{
									continue;
								}

								// Skip sensors.
								bool sensorA = contact.m_fixtureA.IsSensor();
								bool sensorB = contact.m_fixtureB.IsSensor();
								if (sensorA || sensorB)
								{
									continue;
								}

								// Tentatively advance the body to the TOI.
								Sweep backup = other.m_sweep;
								if (!other.HasFlag(BodyFlags.Island))
								{
									other.Advance(minAlpha);
								}

								// Update the contact points
								contact.Update(m_contactManager.m_contactListener);

								// Was the contact disabled by the user?
								if (contact.IsEnabled() == false)
								{
									other.m_sweep = backup;
									other.SynchronizeTransform();
									continue;
								}

								// Are there contact points?
								if (contact.IsTouching() == false)
								{
									other.m_sweep = backup;
									other.SynchronizeTransform();
									continue;
								}

								// Add the contact to the island
								contact.m_flags |= CollisionFlags.Island;
								island.Add(contact);

								// Has the other body already been added to the island?
								if (other.HasFlag(BodyFlags.Island))
								{
									continue;
								}

								// Add the other body to the island.
								other.SetFlag(BodyFlags.Island);

								if (other.m_type != BodyType.Static)
								{
									other.SetAwake(true);
								}

								island.Add(other);
							}
						}
					}

					TimeStep subStep;
					subStep.dt = (1.0f - minAlpha) * step.dt;
					subStep.inv_dt = 1.0f / subStep.dt;
					subStep.dtRatio = 1.0f;
					subStep.positionIterations = 20;
					subStep.velocityIterations = step.velocityIterations;
					subStep.warmStarting = false;
					island.SolveTOI(in subStep, bA.m_islandIndex, bB.m_islandIndex);

					// Reset island flags and synchronize broad-phase proxies.
					for (var i = 0; i < island.m_bodyCount; ++i)
					{
						Body body = island.m_bodies[i];
						body.UnsetFlag(BodyFlags.Island);

						if (body.m_type != BodyType.Dynamic)
						{
							continue;
						}

						body.SynchronizeFixtures();

						// Invalidate all contact TOIs on this displaced body.
						for (ContactEdge ce = body.m_contactList; ce != null; ce = ce.next)
						{
							ce.contact.m_flags &= ~(CollisionFlags.Toi | CollisionFlags.Island);
						}
					}

					// Commit fixture proxy movements to the broad-phase so that new contacts are created.
					// Also, some contacts can be destroyed.
					m_contactManager.FindNewContacts();

					if (m_subStepping)
					{
						m_stepComplete = false;
						break;
					}
				}
			}
		}

		/// <summary>
		///  Take a time step. This performs collision detection, integration,
		///  and constraint solution.
		/// </summary>
		/// <param name="dt">The amount of time to simulate, this should not vary.</param>
		/// <param name="velocityIterations">Iterations for the velocity constraint solver.</param>
		/// <param name="positionIterations">Iterations for the position constraint solver.</param>
		public void Step(float dt, int velocityIterations, int positionIterations)
		{
			if (m_newContacts)
			{
				m_contactManager.FindNewContacts();
				m_newContacts = false;
			}

			m_locked = true;

			TimeStep step;
			step.dt = dt;
			step.velocityIterations = velocityIterations;
			step.positionIterations = positionIterations;
			if (dt > 0.0f)
			{
				step.inv_dt = 1.0f / dt;
			}
			else
			{
				step.inv_dt = 0.0f;
			}

			step.dtRatio = m_inv_dt0 * dt;

			step.warmStarting = m_warmStarting;

			// Update contacts. This is where some contacts are destroyed.
			{
				m_contactManager.Collide();
			}

			// Integrate velocities, solve velocity constraints, and integrate positions.
			if (m_stepComplete && step.dt > 0.0f)
			{
				Solve(step);
			}

			// Handle TOI events.
			if (m_continuousPhysics && step.dt > 0.0f)
			{
				SolveTOI(step);
			}

			if (step.dt > 0.0f)
			{
				m_inv_dt0 = step.inv_dt;
			}

			if (m_clearForces)
			{
				ClearForces();
			}

			m_locked = false;
		}

		public void ClearForces()
		{
			for (Body body = m_bodyList; body != null; body = body.GetNext())
			{
				body.m_force = Vector2.Zero;
				body.m_torque = 0.0f;
			}
		}

		public void QueryAABB(QueryCallback callback, in AABB aabb)
		{
			bool internalCallback(int proxyId)
			{
				var proxy = (FixtureProxy) m_contactManager.m_broadPhase.GetUserData(proxyId);
				return callback(proxy.fixture);
			}

			m_contactManager.m_broadPhase.Query(internalCallback, aabb);
		}

		public int QueryAABB(out Fixture[] fixtures, in AABB aabb, int maxFixtures = 256)
		{
			var result = new Fixture[maxFixtures];

			var i = 0;

			bool internalCallback(int proxyId)
			{
				var proxy = (FixtureProxy) m_contactManager.m_broadPhase.GetUserData(proxyId);
				result[i++] = proxy.fixture;
				return i != maxFixtures;
			}

			m_contactManager.m_broadPhase.Query(internalCallback, aabb);

			fixtures = result;
			return i;
		}

		public void RayCast(RayCastCallback callback, in Vector2 point1, in Vector2 point2)
		{
			float internalCallback(RayCastInput input, int proxyId)
			{
				object userData = m_contactManager.m_broadPhase.GetUserData(proxyId);
				var proxy = (FixtureProxy) userData;
				Fixture fixture = proxy.fixture;
				int index = proxy.childIndex;
				bool hit = fixture.RayCast(out RayCastOutput output, input, index);

				if (hit)
				{
					float fraction = output.fraction;
					Vector2 point = (1f - fraction) * input.p1 + fraction * input.p2;
					callback(fixture, point, output.normal, fraction);
				}

				return input.maxFraction;
			}

			RayCastInput input;
			input.maxFraction = 1.0f;
			input.p1 = point1;
			input.p2 = point2;
			m_contactManager.m_broadPhase.RayCast(internalCallback, in input);
		}

		private void DrawJoint(Joint joint)
		{
			Body b1 = joint.GetBodyA();
			Body b2 = joint.GetBodyB();
			Transform xf1 = b1.GetTransform();
			Transform xf2 = b2.GetTransform();
			Vector2 x1 = xf1.p;
			Vector2 x2 = xf2.p;
			Vector2 p1 = joint.GetAnchorA;
			Vector2 p2 = joint.GetAnchorB;

			var color = new Color(0.5f, 0.8f, 0.8f);

			switch (joint)
			{
				case DistanceJoint j:
					m_debugDraw.DrawSegment(p1, p2, color);
					break;

				case PulleyJoint pulley: {
					Vector2 s1 = pulley.GroundAnchorA;
					Vector2 s2 = pulley.GroundAnchorB;
					m_debugDraw.DrawSegment(s1, p1, color);
					m_debugDraw.DrawSegment(s2, p2, color);
					m_debugDraw.DrawSegment(s1, s2, color);
				}
					break;

				case MouseJoint j:
					// don't draw this
					break;

				default:
					m_debugDraw.DrawSegment(x1, p1, color);
					m_debugDraw.DrawSegment(p1, p2, color);
					m_debugDraw.DrawSegment(x2, p2, color);
					break;
			}
		}

		private void DrawFixture(Fixture fixture, Transform xf, Color color)
		{
			switch (fixture.Shape)
			{
				case CircleShape circle: {
					Vector2 center = Math.Mul(xf, circle.m_p);
					float radius = circle.m_radius;
					var axis = new Vector2(xf.q.M11, xf.q.M21);

					m_debugDraw.DrawSolidCircle(center, radius, axis, color);
				}
					break;

				case PolygonShape poly: {
					int vertexCount = poly.m_count;
					Vector2[] localVertices = poly.m_vertices;

					//Debug.Assert(vertexCount <= Settings.MaxPolygonVertices);
					var vertices = new Vector2[Settings.MaxPolygonVertices];

					for (var i = 0; i < vertexCount; ++i)
					{
						vertices[i] = Math.Mul(xf, localVertices[i]);
					}

					m_debugDraw.DrawSolidPolygon(Vec2.ConvertArray(vertices), vertexCount, color);
				}
					break;

				case EdgeShape edge: {
					m_debugDraw.DrawSegment(Math.Mul(xf, edge.m_vertex1), Math.Mul(xf, edge.m_vertex2), color);
				}
					break;
			}
		}

		public void DrawDebugData()
		{
			DrawFlags flags = m_debugDraw.Flags;
			if ((flags & DrawFlags.Shape) == DrawFlags.Shape)
			{
				for (Body b = m_bodyList; b != null; b = b.GetNext())
				{
					Transform xf = b.GetTransform();
					for (Fixture f = b.GetFixtureList(); f != null; f = f.GetNext())
					{
						if (b.Type() == BodyType.Dynamic && b.m_mass == 0.0f)
							// Bad body
						{
							DrawShape(f, xf, new Color(1.0f, 0.0f, 0.0f));
						}
						else if (b.IsEnabled() == false)
						{
							DrawShape(f, xf, new Color(0.5f, 0.5f, 0.3f));
						}
						else if (b.Type() == BodyType.Static)
						{
							DrawShape(f, xf, new Color(0.5f, 0.9f, 0.5f));
						}
						else if (b.Type() == BodyType.Kinematic)
						{
							DrawShape(f, xf, new Color(0.5f, 0.5f, 0.9f));
						}
						else if (b.IsAwake() == false)
						{
							DrawShape(f, xf, new Color(0.6f, 0.6f, 0.6f));
						}
						else
						{
							DrawShape(f, xf, new Color(0.9f, 0.7f, 0.7f));
						}
					}
				}
			}

			if ((flags & DrawFlags.Joint) == DrawFlags.Joint)
			{
				for (Joint j = m_jointList; j != null; j = j.GetNext())
				{
					j.Draw(m_debugDraw);
				}
			}

			if ((flags & DrawFlags.Pair) == DrawFlags.Pair)
			{
				var color = new Color(0.3f, 0.9f, 0.9f);
				for (Contact c = m_contactManager.m_contactList; c != null; c = c.GetNext())
				{
					Fixture fixtureA = c.GetFixtureA();
					Fixture fixtureB = c.GetFixtureB();
					int indexA = c.GetChildIndexA();
					int indexB = c.GetChildIndexB();
					Vector2 cA = fixtureA.GetAABB(indexA).GetCenter();
					Vector2 cB = fixtureB.GetAABB(indexB).GetCenter();

					m_debugDraw.DrawSegment(cA, cB, color);
				}
			}

			if ((flags & DrawFlags.Aabb) == DrawFlags.Aabb)
			{
				var color = new Color(0.9f, 0.3f, 0.9f);
				BroadPhase bp = m_contactManager.m_broadPhase;

				for (Body b = m_bodyList; b != null; b = b.GetNext())
				{
					if (b.IsEnabled() == false)
					{
						continue;
					}

					for (Fixture f = b.GetFixtureList(); f != null; f = f.GetNext())
					for (var i = 0; i < f.m_proxyCount; ++i)
					{
						FixtureProxy proxy = f.m_proxies[i];
						AABB aabb = bp.GetFatAABB(proxy.proxyId);
						var vs = new Vec2[4];
						vs[0] = new Vec2(aabb.lowerBound.X, aabb.lowerBound.Y);
						vs[1] = new Vec2(aabb.upperBound.X, aabb.lowerBound.Y);
						vs[2] = new Vec2(aabb.upperBound.X, aabb.upperBound.Y);
						vs[3] = new Vec2(aabb.lowerBound.X, aabb.upperBound.Y);

						m_debugDraw.DrawPolygon(vs, 4, color);
					}
				}
			}

			if ((flags & DrawFlags.CenterOfMass) == DrawFlags.CenterOfMass)
			{
				for (Body b = m_bodyList; b != null; b = b.GetNext())
				{
					Transform xf = b.GetTransform();
					xf.p = b.GetWorldCenter();
					m_debugDraw.DrawTransform(xf);
				}
			}
		}

		private void DrawShape(Fixture fixture, in Transform xf, in Color color)
		{
			switch (fixture.Shape)
			{
				case CircleShape circle: {
					Vec2 center = Math.Mul(xf, circle.m_p);
					float radius = circle.m_radius;
					Vec2 axis = Vector2.Transform(new Vector2(1.0f, 0.0f), xf.q); // Math.Mul(xf.q, new Vector2(1.0f, 0.0f));

					m_debugDraw.DrawSolidCircle(center, radius, axis, color);
				}
					break;

				case EdgeShape edge: {
					Vector2 v1 = Math.Mul(xf, edge.m_vertex1);
					Vector2 v2 = Math.Mul(xf, edge.m_vertex2);
					m_debugDraw.DrawSegment(v1, v2, color);

					if (!edge.m_oneSided)
					{
						m_debugDraw.DrawPoint(v1, 4f, color);
						m_debugDraw.DrawPoint(v2, 4f, color);
					}
				}
					break;

				case ChainShape chain: {
					int count = chain.m_count;
					Vector2[] vertices = chain.m_vertices;

					Vector2 v1 = Math.Mul(xf, vertices[0]);
					m_debugDraw.DrawPoint(v1, 4.0f, color);

					for (var i = 1; i < count; ++i)
					{
						Vector2 v2 = Math.Mul(xf, vertices[i]);
						m_debugDraw.DrawSegment(v1, v2, color);
						v1 = v2;
					}
				}
					break;

				case PolygonShape poly: {
					int vertexCount = poly.m_count;
					var vertices = new Vec2[Settings.MaxPolygonVertices];

					for (var i = 0; i < vertexCount; ++i)
					{
						vertices[i] = Math.Mul(xf, poly.m_vertices[i]);
					}

					m_debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
				}
					break;
			}
		}
	}
}