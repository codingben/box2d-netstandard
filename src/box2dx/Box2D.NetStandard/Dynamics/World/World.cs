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
using Box2D.NetStandard.Dynamics.Joints.Pulley;
using Box2D.NetStandard.Dynamics.World.Callbacks;
using Math = Box2D.NetStandard.Common.Math;

namespace Box2D.NetStandard.Dynamics.World {
  /// <summary>
  /// The world class manages all physics entities, dynamic simulation,
  /// and asynchronous queries.
  /// </summary>
  public class World {
    //internal BroadPhase     _broadPhase;
    internal ContactManager _contactManager;

    private Body  _bodyList;
    private Joint _jointList;

    private int _bodyCount;
    private int _jointCount;

    private Vector2 _gravity;
    private bool    _allowSleep;

    private DestructionListener _destructionListener;
    private DebugDraw           _debugDraw;

    private float _inv_dt0;

    internal bool _newContacts;
    private  bool _locked;
    private  bool _clearForces;

    // These are for debugging the solver
    private bool _warmStarting;
    private bool _continuousPhysics;
    private bool _subStepping;

    private bool _stepComplete;

    private Action DrawDebugDataStub = () => { };

    /// <summary>
    /// Get\Set global gravity vector.
    /// </summary>
    public Vector2 Gravity {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => _gravity;
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      set => _gravity = value;
    }

    /// <summary>
    /// Get the world body list. With the returned body, use Body.GetNext to get
    /// the next body in the world list. A null body indicates the end of the list.
    /// </summary>
    /// <returns>The head of the world body list.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Body GetBodyList() => _bodyList;

    /// <summary>
    /// Get the world joint list. With the returned joint, use Joint.GetNext to get
    /// the next joint in the world list. A null joint indicates the end of the list.
    /// </summary>
    /// <returns>The head of the world joint list.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Joint GetJointList() => _jointList;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Contact GetContactList() => _contactManager.m_contactList;

    /// <summary>
    /// Get the number of bodies.
    /// </summary>
    /// <returns></returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GetBodyCount() => _bodyCount;

    /// <summary>
    /// Get the number joints.
    /// </summary>
    /// <returns></returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GetJointCount() => _jointCount;

    /// <summary>
    /// Get the number of contacts (each may have 0 or more contact points).
    /// </summary>
    /// <returns></returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GetContactCount() => _contactManager.m_contactCount;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetGravity(in Vector2 gravity) => _gravity = gravity;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2 GetGravity() => _gravity;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsLocked() => _locked;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool GetAutoClearGorces() => _clearForces;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal ContactManager GetContactManager() => _contactManager;

    public World() : this(new Vector2(0, -10)) { }

    /// <summary>
    /// Construct a world object.
    /// </summary>
    /// <param name="worldAABB">A bounding box that completely encompasses all your shapes.</param>
    /// <param name="gravity">The world gravity vector.</param>
    /// <param name="doSleep">Improve performance by not simulating inactive bodies.</param>
    public World(Vector2 gravity) {
      _destructionListener = null;
      _debugDraw           = null;

      _bodyList  = null;
      _jointList = null;

      _bodyCount  = 0;
      _jointCount = 0;

      _warmStarting      = true;
      _continuousPhysics = true;
      _subStepping       = false;

      _stepComplete = true;
      _allowSleep   = true;
      _gravity      = gravity;

      _newContacts = false;
      _locked      = false;
      _clearForces = true;

      _inv_dt0 = 0.0f;

      _contactManager = new ContactManager();
    }


    /// <summary>
    /// Register a destruction listener.
    /// </summary>
    /// <param name="listener"></param>
    public void SetDestructionListener(DestructionListener listener) => _destructionListener = listener;


    /// <summary>
    /// Register a contact filter to provide specific control over collision.
    /// Otherwise the default filter is used (b2_defaultFilter).
    /// </summary>
    /// <param name="filter"></param>
    public void SetContactFilter(ContactFilter filter) => _contactManager.m_contactFilter = filter;

    /// <summary>
    /// Register a contact event listener
    /// </summary>
    /// <param name="listener"></param>
    public void SetContactListener(ContactListener listener) => _contactManager.m_contactListener = listener;

    /// <summary>
    /// Register a routine for debug drawing. The debug draw functions are called
    /// inside the World.Step method, so make sure your renderer is ready to
    /// consume draw commands when you call Step().
    /// </summary>
    /// <param name="debugDraw"></param>
    public void SetDebugDraw(DebugDraw debugDraw) {
      _debugDraw        = debugDraw;
      DrawDebugDataStub = DrawDebugData;
    }

    /// <summary>
    /// Create a rigid body given a definition. No reference to the definition
    /// is retained.
    /// @warning This function is locked during callbacks.
    /// </summary>
    /// <param name="def"></param>
    /// <returns></returns>
    public Body CreateBody(BodyDef def) {
      //Debug.Assert(_locked == false);
      if (_locked == true) {
        return null;
      }

      Body b = new Body(def, this);

      // Add to world doubly linked list.
      b._prev = null;
      b._next = _bodyList;
      if (_bodyList != null) {
        _bodyList._prev = b;
      }

      _bodyList = b;
      ++_bodyCount;

      return b;
    }

    /// <summary>
    /// Destroy a rigid body given a definition. No reference to the definition
    /// is retained. This function is locked during callbacks.
    /// @warning This automatically deletes all associated shapes and joints.
    /// @warning This function is locked during callbacks.
    /// </summary>
    /// <param name="b"></param>
    public void DestroyBody(Body b) {
      //Debug.Assert(_bodyCount > 0);
      //Debug.Assert(_locked    == false);
      if (_locked == true) {
        return;
      }

      // Delete the attached joints.
      JointEdge je = b._jointList;
      while (je != null) {
        JointEdge je0 = je;
        je = je.next;

        _destructionListener?.SayGoodbye(je0.joint);

        DestroyJoint(je0.joint);

        b._jointList = je;
      }

      b._jointList = null;

      ContactEdge ce = b._contactList;
      while (ce != null) {
        ContactEdge ce0 = ce;
        ce = ce.next;
        _contactManager.Destroy(ce0.contact);
      }

      b._contactList = null;

      // Delete the attached fixtures. This destroys broad-phase
      // proxies.
      Fixture f = b._fixtureList;
      while (f != null) {
        Fixture f0 = f;
        f = f.m_next;

        _destructionListener?.SayGoodbye(f0);

        f0.DestroyProxies(_contactManager.m_broadPhase);

        b._fixtureList  =  f;
        b._fixtureCount -= 1;
      }

      b._fixtureList  = null;
      b._fixtureCount = 0;

      // Remove world body list.
      if (b._prev != null) {
        b._prev._next = b._next;
      }

      if (b._next != null) {
        b._next._prev = b._prev;
      }

      if (b == _bodyList) {
        _bodyList = b._next;
      }

      --_bodyCount;
      b = null;
    }

    /// <summary>
    /// Create a joint to constrain bodies together. No reference to the definition
    /// is retained. This may cause the connected bodies to cease colliding.
    /// @warning This function is locked during callbacks.
    /// </summary>
    /// <param name="def"></param>
    /// <returns></returns>
    public Joint CreateJoint(JointDef def) {
      //Debug.Assert(_locked == false);

      if (_locked) return null;

      Joint j = Joint.Create(def);

      // Connect to the world list.
      j._prev = null;
      j._next = _jointList;
      if (_jointList != null) {
        _jointList._prev = j;
      }

      _jointList = j;
      ++_jointCount;

      // Connect to the bodies' doubly linked lists.
      j._edgeA.joint = j;
      j._edgeA.other = j._bodyB;
      j._edgeA.Prev  = null;
      j._edgeA.next  = j._bodyA._jointList;
      if (j._bodyA._jointList != null)
        j._bodyA._jointList.Prev = j._edgeA;
      j._bodyA._jointList = j._edgeA;

      j._edgeB.joint = j;
      j._edgeB.other = j._bodyA;
      j._edgeB.Prev  = null;
      j._edgeB.next  = j._bodyB._jointList;
      if (j._bodyB._jointList != null)
        j._bodyB._jointList.Prev = j._edgeB;
      j._bodyB._jointList = j._edgeB;

      Body bodyA = def.bodyA;
      Body bodyB = def.bodyB;

      // If the joint prevents collisions, then flag any contacts for filtering.
      if (def.collideConnected == false) {
        ContactEdge edge = bodyB._contactList;
        while (edge != null) {
          if (edge.other == bodyA) {
            // Flag the contact for filtering at the next time step (where either
            // body is awake).
            edge.contact.FlagForFiltering();
          }

          edge = edge.next;
        }
      }

      // Note: creating a joint doesn't wake the bodies.

      return j;
    }

    /// <summary>
    /// Destroy a joint. This may cause the connected bodies to begin colliding.
    /// @warning This function is locked during callbacks.
    /// </summary>
    /// <param name="j"></param>
    public void DestroyJoint(Joint j) {
      //Debug.Assert(_locked == false);
      if (_locked) return;

      bool collideConnected = j._collideConnected;

      // Remove from the doubly linked list.
      if (j._prev != null) {
        j._prev._next = j._next;
      }

      if (j._next != null) {
        j._next._prev = j._prev;
      }

      if (j == _jointList) {
        _jointList = j._next;
      }

      // Disconnect from island graph.
      Body bodyA = j._bodyA;
      Body bodyB = j._bodyB;

      // Wake up connected bodies.
      bodyA.SetAwake(true);
      bodyB.SetAwake(true);

      // Remove from body 1.
      if (j._edgeA.Prev != null) {
        j._edgeA.Prev.next = j._edgeA.next;
      }

      if (j._edgeA.next != null) {
        j._edgeA.next.Prev = j._edgeA.Prev;
      }

      if (j._edgeA == bodyA._jointList) {
        bodyA._jointList = j._edgeA.next;
      }

      j._edgeA.Prev = null;
      j._edgeA.next = null;

      // Remove from body 2
      if (j._edgeB.Prev != null) {
        j._edgeB.Prev.next = j._edgeB.next;
      }

      if (j._edgeB.next != null) {
        j._edgeB.next.Prev = j._edgeB.Prev;
      }

      if (j._edgeB == bodyB._jointList) {
        bodyB._jointList = j._edgeB.next;
      }

      j._edgeB.Prev = null;
      j._edgeB.next = null;

      //Debug.Assert(_jointCount > 0);
      --_jointCount;

      // If the joint prevents collisions, then flag any contacts for filtering.
      if (collideConnected == false) {
        ContactEdge edge = bodyB._contactList;
        while (edge != null) {
          if (edge.other == bodyA) {
            // Flag the contact for filtering at the next time step (where either
            // body is awake).
            edge.contact.FlagForFiltering();
          }

          edge = edge.next;
        }
      }
    }

    public void SetAllowSleeping(bool flag) {
      if (flag == _allowSleep) return;

      _allowSleep = flag;
      if (!_allowSleep) {
        for (Body b = _bodyList; b != null; b = b._next) {
          b.SetAwake(true);
        }
      }
    }


    // Find islands, integrate and solve constraints, solve position constraints
    private void Solve(TimeStep step) {
      // Size the island for the worst case.
      Island island = new Island(_bodyCount,
                                 _contactManager.m_contactCount,
                                 _jointCount,
                                 _contactManager.m_contactListener);

      // Clear all the island flags.
      for (Body b = _bodyList; b != null; b = b._next) {
        b.UnsetFlag(BodyFlags.Island);
      }

      for (Contact c = _contactManager.m_contactList; c != null; c = c.m_next) {
        c.m_flags &= ~CollisionFlags.Island;
      }

      for (Joint j = _jointList; j != null; j = j._next) {
        j._islandFlag = false;
      }

      // Build and simulate all awake islands.
      int stackSize = _bodyCount;
      //Stack<Body> stack = new Stack<Body>(_bodyCount);
      //Body stack     = (b2Body**) m_stackAllocator.Allocate(stackSize * sizeof(b2Body*));
      Body[] stack = new Body[_bodyCount];
      for (Body seed = _bodyList; seed != null; seed = seed._next) {
        if (seed.HasFlag(BodyFlags.Island)) {
          continue;
        }

        if (seed.IsAwake() == false || seed.IsEnabled() == false) {
          continue;
        }

        // The seed can be dynamic or kinematic.
        if (seed._type == BodyType.Static) {
          continue;
        }

        // Reset island and stack.
        island.Clear();
        int stackCount = 0;
        stack[stackCount++] = seed;
        seed.SetFlag(BodyFlags.Island);

        // Perform a depth first search (DFS) on the constraint graph.
        while (stackCount > 0) {
          // Grab the next body off the stack and add it to the island.
          Body b = stack[--stackCount];
          ////Debug.Assert(b.IsEnabled() == true);
          island.Add(b);

          // Make sure the body is awake (without resetting sleep timer).
          b.SetFlag(BodyFlags.Awake);

          // To keep islands as small as possible, we don't
          // propagate islands across static bodies.
          if (b._type == BodyType.Static) {
            continue;
          }

          // Search all contacts connected to this body.
          for (ContactEdge ce = b._contactList; ce != null; ce = ce.next) {
            Contact contact = ce.contact;

            // Has this contact already been added to an island?
            if ((contact.m_flags & CollisionFlags.Island) == CollisionFlags.Island) {
              continue;
            }

            // Is this contact solid and touching?
            if (contact.IsEnabled()  == false ||
                contact.IsTouching() == false) {
              continue;
            }

            // Skip sensors.
            bool sensorA = contact.m_fixtureA.IsSensor();
            bool sensorB = contact.m_fixtureB.IsSensor();
            if (sensorA || sensorB) {
              continue;
            }

            island.Add(contact);
            contact.m_flags |= CollisionFlags.Island;

            Body other = ce.other;

            // Was the other body already added to this island?
            if (other.HasFlag(BodyFlags.Island)) {
              continue;
            }

            //Debug.Assert(stackCount < stackSize);
            stack[stackCount++] = other;
            other.SetFlag(BodyFlags.Island);
          }

          // Search all joints connect to this body.
          for (JointEdge je = b._jointList; je != null; je = je.next) {
            if (je.joint._islandFlag == true) {
              continue;
            }

            Body other = je.other;

            // Don't simulate joints connected to disabled bodies.
            if (other.IsEnabled() == false) {
              continue;
            }

            island.Add(je.joint);
            je.joint._islandFlag = true;

            if (other.HasFlag(BodyFlags.Island)) {
              continue;
            }

            //Debug.Assert(stackCount < stackSize);
            stack[stackCount++] = other;
            other.SetFlag(BodyFlags.Island);
          }
        }

        island.Solve(step, _gravity, _allowSleep);

        // Post solve cleanup.
        for (int i = 0; i < island._bodyCount; ++i) {
          // Allow static bodies to participate in other islands.
          Body b = island._bodies[i];
          if (b._type == BodyType.Static) {
            b.UnsetFlag(BodyFlags.Island);
          }
        }
      }

      stack = null;

      {
        // Synchronize fixtures, check for out of range bodies.
        for (Body b = _bodyList; b != null; b = b.GetNext()) {
          // If a body was not in an island then it did not move.
          if (!(b.HasFlag(BodyFlags.Island))) {
            continue;
          }

          if (b._type == BodyType.Static) {
            continue;
          }

          // Update fixtures (for broad-phase).
          b.SynchronizeFixtures();
        }

        // Look for new contacts.
        _contactManager.FindNewContacts();
      }
    }


    // Find TOI contacts and solve them.
    private void SolveTOI(in TimeStep step) {
      Island island = new Island(2 * Settings.MaxTOIContacts, Settings.MaxTOIContacts, 0,
                                 _contactManager.m_contactListener);

      if (_stepComplete) {
        for (Body b = _bodyList; b != null; b = b._next) {
          b.UnsetFlag(BodyFlags.Island);
          b._sweep.alpha0 = 0.0f;
        }

        for (Contact c = _contactManager.m_contactList; c != null; c = c.m_next) {
          // Invalidate TOI
          c.m_flags   &= ~(CollisionFlags.Toi | CollisionFlags.Island);
          c._toiCount =  0;
          c._toi      =  1.0f;
        }
      }

      // Find TOI events and solve them.
      for (;;) {
        // Find the first TOI.
        Contact minContact = null;
        float   minAlpha   = 1.0f;

        for (Contact c = _contactManager.m_contactList; c != null; c = c.m_next) {
          // Is this contact disabled?
          if (c.Enabled == false) {
            continue;
          }

          // Prevent excessive sub-stepping.
          if (c._toiCount > Settings.MaxSubSteps) {
            continue;
          }

          float alpha = 1.0f;
          if ((c.m_flags & CollisionFlags.Toi) == CollisionFlags.Toi) {
            // This contact has a valid cached TOI.
            alpha = c._toi;
          }
          else {
            Fixture fA = c.FixtureA;
            Fixture fB = c.FixtureB;

            // Is there a sensor?
            if (fA.IsSensor() || fB.IsSensor()) {
              continue;
            }

            Body bA = fA.Body;
            Body bB = fB.Body;

            bool activeA = bA.IsAwake() && bA._type != BodyType.Static;
            bool activeB = bB.IsAwake() && bB._type != BodyType.Static;

            // Is at least one body active (awake and dynamic or kinematic)?
            if (activeA == false && activeB == false) {
              continue;
            }

            bool collideA = bA.IsBullet() || bA._type != BodyType.Dynamic;
            bool collideB = bB.IsBullet() || bB._type != BodyType.Dynamic;

            // Are these two non-bullet dynamic bodies?
            if (collideA == false && collideB == false) {
              continue;
            }

            // Compute the TOI for this contact.
            // Put the sweeps onto the same time interval.
            float alpha0 = bA._sweep.alpha0;

            if (bA._sweep.alpha0 < bB._sweep.alpha0) {
              bA._sweep.Advance(alpha0 = bB._sweep.alpha0);
            }
            else if (bB._sweep.alpha0 < bA._sweep.alpha0) {
              bB._sweep.Advance(alpha0);
            }

            //Debug.Assert(alpha0 < 1.0f);

            // Compute the time of impact in interval [0, minTOI]
            TOIInput input;
            input.proxyA = new DistanceProxy();
            input.proxyA.Set(fA.Shape, c.ChildIndexA);
            input.proxyB = new DistanceProxy();
            input.proxyB.Set(fB.Shape, c.ChildIndexB);
            input.sweepA = bA._sweep;
            input.sweepB = bB._sweep;
            input.tMax   = 1.0f;

            TOI.TimeOfImpact(out TOIOutput output, in input);

            // Beta is the fraction of the remaining portion of the .
            if (output.state == TOIOutputState.Touching) {
              alpha = MathF.Min(alpha0 + (1.0f - alpha0) * output.t, 1.0f);
            }
            else {
              alpha = 1.0f;
            }

            c._toi    =  alpha;
            c.m_flags |= CollisionFlags.Toi;
          }

          if (alpha < minAlpha) {
            // This is the minimum TOI found so far.
            minContact = c;
            minAlpha   = alpha;
          }
        }

        if (minContact == null || 1.0f - 10.0f * Settings.FLT_EPSILON < minAlpha) {
          // No more TOI events. Done!
          _stepComplete = true;
          break;
        }

        {
          // Advance the bodies to the TOI.
          Fixture fA = minContact.m_fixtureA;
          Fixture fB = minContact.m_fixtureB;
          Body    bA = fA.Body;
          Body    bB = fB.Body;

          Sweep backup1 = bA._sweep;
          Sweep backup2 = bB._sweep;

          bA.Advance(minAlpha);
          bB.Advance(minAlpha);

          // The TOI contact likely has some new contact points.
          minContact.Update(_contactManager.m_contactListener);
          minContact.m_flags &= ~CollisionFlags.Toi;
          ++minContact._toiCount;

          // Is the contact solid?
          if (minContact.IsEnabled() == false || minContact.IsTouching() == false) {
            // Restore the sweeps.
            minContact.SetEnabled(false);
            bA._sweep = backup1;
            bB._sweep = backup2;
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

          for (int i = 0; i < 2; ++i) {
            Body body = bodies[i];
            if (body._type == BodyType.Dynamic) {
              for (ContactEdge ce = body._contactList; ce != null; ce = ce.next) {
                if (island._bodyCount == island._bodyCapacity) {
                  break;
                }

                if (island._contactCount == island._contactCapacity) {
                  break;
                }

                Contact contact = ce.contact;

                // Has this contact already been added to the island?
                if ((contact.m_flags & CollisionFlags.Island) == CollisionFlags.Island) {
                  continue;
                }

                // Only add static, kinematic, or bullet bodies.
                Body other = ce.other;
                if (other._type     == BodyType.Dynamic &&
                    body.IsBullet() == false            && other.IsBullet() == false) {
                  continue;
                }

                // Skip sensors.
                bool sensorA = contact.m_fixtureA.IsSensor();
                bool sensorB = contact.m_fixtureB.IsSensor();
                if (sensorA || sensorB) {
                  continue;
                }

                // Tentatively advance the body to the TOI.
                Sweep backup = other._sweep;
                if (!(other.HasFlag(BodyFlags.Island))) {
                  other.Advance(minAlpha);
                }

                // Update the contact points
                contact.Update(_contactManager.m_contactListener);

                // Was the contact disabled by the user?
                if (contact.IsEnabled() == false) {
                  other._sweep = backup;
                  other.SynchronizeTransform();
                  continue;
                }

                // Are there contact points?
                if (contact.IsTouching() == false) {
                  other._sweep = backup;
                  other.SynchronizeTransform();
                  continue;
                }

                // Add the contact to the island
                contact.m_flags |= CollisionFlags.Island;
                island.Add(contact);

                // Has the other body already been added to the island?
                if (other.HasFlag(BodyFlags.Island)) {
                  continue;
                }

                // Add the other body to the island.
                other.SetFlag(BodyFlags.Island);

                if (other._type != BodyType.Static) {
                  other.SetAwake(true);
                }

                island.Add(other);
              }
            }
          }

          TimeStep subStep;
          subStep.dt                 = (1.0f - minAlpha) * step.dt;
          subStep.inv_dt             = 1.0f              / subStep.dt;
          subStep.dtRatio            = 1.0f;
          subStep.positionIterations = 20;
          subStep.velocityIterations = step.velocityIterations;
          subStep.warmStarting       = false;
          island.SolveTOI(in subStep, bA._islandIndex, bB._islandIndex);

          // Reset island flags and synchronize broad-phase proxies.
          for (int i = 0; i < island._bodyCount; ++i) {
            Body body = island._bodies[i];
            body.UnsetFlag(BodyFlags.Island);

            if (body._type != BodyType.Dynamic) {
              continue;
            }

            body.SynchronizeFixtures();

            // Invalidate all contact TOIs on this displaced body.
            for (ContactEdge ce = body._contactList; ce != null; ce = ce.next) {
              ce.contact.m_flags &= ~(CollisionFlags.Toi | CollisionFlags.Island);
            }
          }

          // Commit fixture proxy movements to the broad-phase so that new contacts are created.
          // Also, some contacts can be destroyed.
          _contactManager.FindNewContacts();

          if (_subStepping) {
            _stepComplete = false;
            break;
          }
        }
      }
    }

    /// <summary>
    /// Take a time step. This performs collision detection, integration,
    /// and constraint solution.
    /// </summary>
    /// <param name="dt">The amount of time to simulate, this should not vary.</param>
    /// <param name="iterations">For the velocity constraint solver.</param>
    /// <param name="iterations">For the positionconstraint solver.</param>
    public void Step(float dt, int velocityIterations, int positionIterations) {
      if (_newContacts) {
        _contactManager.FindNewContacts();
        _newContacts = false;
      }

      _locked = true;

      TimeStep step;
      step.dt                 = dt;
      step.velocityIterations = velocityIterations;
      step.positionIterations = positionIterations;
      if (dt > 0.0f) {
        step.inv_dt = 1.0f / dt;
      }
      else {
        step.inv_dt = 0.0f;
      }

      step.dtRatio = _inv_dt0 * dt;

      step.warmStarting = _warmStarting;

      // Update contacts. This is where some contacts are destroyed.
      {
        _contactManager.Collide();
      }

      // Integrate velocities, solve velocity constraints, and integrate positions.
      if (_stepComplete && step.dt > 0.0f) {
        Solve(step);
      }

      // Handle TOI events.
      if (_continuousPhysics && step.dt > 0.0f) {
        SolveTOI(step);
      }

      if (step.dt > 0.0f) {
        _inv_dt0 = step.inv_dt;
      }

      if (_clearForces) {
        ClearForces();
      }

      _locked = false;
    }

    public void ClearForces() {
      for (Body body = _bodyList; body != null; body = body.GetNext()) {
        body._force  = Vector2.Zero;
        body._torque = 0.0f;
      }
    }

    public delegate bool QueryCallback(Fixture fixture);

    public void QueryAABB(QueryCallback callback, in AABB aabb) {
      bool internalCallback(int proxyId) {
        FixtureProxy proxy = (FixtureProxy) _contactManager.m_broadPhase.GetUserData(proxyId);
        return callback(proxy.fixture);
      }

      _contactManager.m_broadPhase.Query(internalCallback, aabb);
    }

    public int QueryAABB(out Fixture[] fixtures, in AABB aabb, int maxFixtures = 256) {
      Fixture[] result = new Fixture[maxFixtures];

      int i = 0;

      bool internalCallback(int proxyId) {
        FixtureProxy proxy = (FixtureProxy) _contactManager.m_broadPhase.GetUserData(proxyId);
        result[i++] = proxy.fixture;
        return i != maxFixtures;
      }

      _contactManager.m_broadPhase.Query(internalCallback, aabb);

      fixtures = result;
      return i;
    }

    public delegate void RayCastCallback(Fixture fixture, Vector2 point, Vector2 normal, float fraction);

    public void RayCast(RayCastCallback callback, in Vector2 point1, in Vector2 point2) {
      float internalCallback(RayCastInput input, int proxyId) {
        object       userData = _contactManager.m_broadPhase.GetUserData(proxyId);
        FixtureProxy proxy    = (FixtureProxy) userData;
        Fixture      fixture  = proxy.fixture;
        int          index    = proxy.childIndex;
        bool         hit      = fixture.RayCast(out RayCastOutput output, input, index);

        if (hit) {
          float   fraction = output.fraction;
          Vector2 point    = (1f - fraction) * input.p1 + fraction * input.p2;
          callback(fixture, point, output.normal, fraction);
        }

        return input.maxFraction;
      }

      RayCastInput input;
      input.maxFraction = 1.0f;
      input.p1          = point1;
      input.p2          = point2;
      _contactManager.m_broadPhase.RayCast(internalCallback, in input);
    }

    private void DrawJoint(Joint joint) {
      Body      b1  = joint.GetBodyA();
      Body      b2  = joint.GetBodyB();
      Transform xf1 = b1.GetTransform();
      Transform xf2 = b2.GetTransform();
      Vector2   x1  = xf1.p;
      Vector2   x2  = xf2.p;
      Vector2   p1  = joint.GetAnchorA;
      Vector2   p2  = joint.GetAnchorB;

      Color color = new Color(0.5f, 0.8f, 0.8f);

      switch (joint.Type) {
        case JointType.DistanceJoint:
          _debugDraw.DrawSegment(p1, p2, color);
          break;

        case JointType.PulleyJoint: {
          PulleyJoint pulley = (PulleyJoint) joint;
          Vector2     s1     = pulley.GroundAnchorA;
          Vector2     s2     = pulley.GroundAnchorB;
          _debugDraw.DrawSegment(s1, p1, color);
          _debugDraw.DrawSegment(s2, p2, color);
          _debugDraw.DrawSegment(s1, s2, color);
        }
          break;

        case JointType.MouseJoint:
          // don't draw this
          break;

        default:
          _debugDraw.DrawSegment(x1, p1, color);
          _debugDraw.DrawSegment(p1, p2, color);
          _debugDraw.DrawSegment(x2, p2, color);
          break;
      }
    }

    private void DrawFixture(Fixture fixture, Transform xf, Color color, bool core) {
#warning "the core argument is not used, the coreColor variable is also not used"
      Color coreColor = new Color(0.9f, 0.6f, 0.6f);

      switch (fixture.Type) {
        case ShapeType.Circle: {
          CircleShape circle = (CircleShape) fixture.Shape;

          Vector2 center = Math.Mul(xf, circle.m_p);
          float   radius = circle.m_radius;
          Vector2 axis   = new Vector2(xf.q.M11, xf.q.M21);

          _debugDraw.DrawSolidCircle(center, radius, axis, color);
        }
          break;

        case ShapeType.Polygon: {
          PolygonShape poly          = (PolygonShape) fixture.Shape;
          int          vertexCount   = poly.m_count;
          Vector2[]    localVertices = poly.m_vertices;

          //Debug.Assert(vertexCount <= Settings.MaxPolygonVertices);
          Vector2[] vertices = new Vector2[Settings.MaxPolygonVertices];

          for (int i = 0; i < vertexCount; ++i) {
            vertices[i] = Math.Mul(xf, localVertices[i]);
          }

          _debugDraw.DrawSolidPolygon(Vec2.ConvertArray(vertices), vertexCount, color);
        }
          break;

        case ShapeType.Edge: {
          EdgeShape edge = (EdgeShape) fixture.Shape;

          _debugDraw.DrawSegment(Math.Mul(xf, edge.m_vertex1), Math.Mul(xf, edge.m_vertex2), color);
        }
          break;
      }
    }

    public void DrawDebugData() {
      var flags = _debugDraw.Flags;
      if ((flags & DrawFlags.Shape) == DrawFlags.Shape) {
        for (Body b = _bodyList; b != null; b = b.GetNext()) {
          Transform xf = b.GetTransform();
          for (Fixture f = b.GetFixtureList(); f != null; f = f.GetNext()) {
            if (b.Type() == BodyType.Dynamic && b._mass == 0.0f) {
              // Bad body
              DrawShape(f, xf, new Color(1.0f, 0.0f, 0.0f));
            }
            else if (b.IsEnabled() == false) {
              DrawShape(f, xf, new Color(0.5f, 0.5f, 0.3f));
            }
            else if (b.Type() == BodyType.Static) {
              DrawShape(f, xf, new Color(0.5f, 0.9f, 0.5f));
            }
            else if (b.Type() == BodyType.Kinematic) {
              DrawShape(f, xf, new Color(0.5f, 0.5f, 0.9f));
            }
            else if (b.IsAwake() == false) {
              DrawShape(f, xf, new Color(0.6f, 0.6f, 0.6f));
            }
            else {
              DrawShape(f, xf, new Color(0.9f, 0.7f, 0.7f));
            }
          }
        }
      }

      if ((flags & DrawFlags.Joint) == DrawFlags.Joint) {
        for (Joint j = _jointList; j != null; j = j.GetNext()) {
          j.Draw(_debugDraw);
        }
      }

      if ((flags & DrawFlags.Pair) == DrawFlags.Pair) {
        Color color = new Color(0.3f, 0.9f, 0.9f);
        for (Contact c = _contactManager.m_contactList; c != null; c = c.GetNext()) {
          Fixture fixtureA = c.GetFixtureA();
          Fixture fixtureB = c.GetFixtureB();
          int     indexA   = c.GetChildIndexA();
          int     indexB   = c.GetChildIndexB();
          Vector2 cA       = fixtureA.GetAABB(indexA).GetCenter();
          Vector2 cB       = fixtureB.GetAABB(indexB).GetCenter();

          _debugDraw.DrawSegment(cA, cB, color);
        }
      }

      if ((flags & DrawFlags.Aabb) == DrawFlags.Aabb) {
        Color      color = new Color(0.9f, 0.3f, 0.9f);
        BroadPhase bp    = _contactManager.m_broadPhase;

        for (Body b = _bodyList; b != null; b = b.GetNext()) {
          if (b.IsEnabled() == false) {
            continue;
          }

          for (Fixture f = b.GetFixtureList(); f != null; f = f.GetNext()) {
            for (int i = 0; i < f.m_proxyCount; ++i) {
              FixtureProxy proxy = f.m_proxies[i];
              AABB         aabb  = bp.GetFatAABB(proxy.proxyId);
              Vec2[]       vs    = new Vec2[4];
              vs[0] = new Vec2(aabb.lowerBound.X, aabb.lowerBound.Y);
              vs[1] = new Vec2(aabb.upperBound.X, aabb.lowerBound.Y);
              vs[2] = new Vec2(aabb.upperBound.X, aabb.upperBound.Y);
              vs[3] = new Vec2(aabb.lowerBound.X, aabb.upperBound.Y);

              _debugDraw.DrawPolygon(vs, 4, color);
            }
          }
        }
      }

      if ((flags & DrawFlags.CenterOfMass) == DrawFlags.CenterOfMass) {
        for (Body b = _bodyList; b != null; b = b.GetNext()) {
          Transform xf = b.GetTransform();
          xf.p = b.GetWorldCenter();
          _debugDraw.DrawTransform(xf);
        }
      }
    }

    private void DrawShape(Fixture fixture, in Transform xf, in Color color) {
      switch (fixture.Type) {
        case ShapeType.Circle: {
          CircleShape circle = (CircleShape) fixture.Shape;

          Vec2  center = Math.Mul(xf, circle.m_p);
          float radius = circle.m_radius;
          Vec2  axis   = Vector2.Transform(new Vector2(1.0f, 0.0f), xf.q); // Math.Mul(xf.q, new Vector2(1.0f, 0.0f));

          _debugDraw.DrawSolidCircle(center, radius, axis, color);
        }
          break;

        case ShapeType.Edge: {
          EdgeShape edge = (EdgeShape) fixture.Shape;
          Vector2   v1   = Math.Mul(xf, edge.m_vertex1);
          Vector2   v2   = Math.Mul(xf, edge.m_vertex2);
          _debugDraw.DrawSegment(v1, v2, color);
        }
          break;

        case ShapeType.Chain: {
          ChainShape chain    = (ChainShape) fixture.Shape;
          int        count    = chain.m_count;
          Vector2[]  vertices = chain.m_vertices;

          Color ghostColor = new Color(0.75f * color.R, 0.75f * color.G, 0.75f * color.B, color.A);

          Vector2 v1 = Math.Mul(xf, vertices[0]);
          _debugDraw.DrawPoint(v1, 4.0f, color);

          if (chain.m_hasPrevVertex) {
            Vector2 vp = Math.Mul(xf, chain.m_prevVertex.Value);
            _debugDraw.DrawSegment(vp, v1, ghostColor);
            _debugDraw.DrawCircle(vp, 0.1f, ghostColor);
          }

          for (int i = 1; i < count; ++i) {
            Vector2 v2 = Math.Mul(xf, vertices[i]);
            _debugDraw.DrawSegment(v1, v2, color);
            _debugDraw.DrawPoint(v2, 4.0f, color);
            v1 = v2;
          }

          if (chain.m_hasNextVertex) {
            Vector2 vn = Math.Mul(xf, chain.m_nextVertex.Value);
            _debugDraw.DrawSegment(v1, vn, ghostColor);
            _debugDraw.DrawCircle(vn, 0.1f, ghostColor);
          }
        }
          break;

        case ShapeType.Polygon: {
          PolygonShape poly        = (PolygonShape) fixture.Shape;
          int          vertexCount = poly.m_count;
          Vec2[]       vertices    = new Vec2[Settings.MaxPolygonVertices];

          for (int i = 0; i < vertexCount; ++i) {
            vertices[i] = Math.Mul(xf, poly.m_vertices[i]);
          }

          _debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
        }
          break;

        default:
          break;
      }
    }
  }
}