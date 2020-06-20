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

/*
Position Correction Notes
=========================
I tried the several algorithms for position correction of the 2D revolute joint.
I looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.

Here are the algorithms:

Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.

Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.

Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is re-computed for each constraint and the positions are updated
after the constraint is solved. The radius vectors (aka Jacobians) are
re-computed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than b2_linearSlop.

Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
each time a constraint is solved.

Here are the results:
Baumgarte - this is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and false bounce. I used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.

Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. I used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on it.

Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.

Full NGS - stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.

Recommendations
Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefit over Baumgarte is small.

Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other constraint
types, especially scalar constraints where the effective mass is a scalar.

This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. I don't think we can escape Baumgarte, especially in highly
demanding cases where high constraint fidelity is not needed.

Full NGS is robust and easy on the eyes. I recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic. The number of NGS iterations can be reduced
for better performance without harming robustness much.

Each joint in a can be handled differently in the position solver. So I recommend
a system where the user can select the algorithm on a per joint basis. I would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.
*/

/*
Cache Performance

The Box2D solvers are dominated by cache misses. Data structures are designed
to increase the number of cache hits. Much of misses are due to random access
to body data. The constraint structures are iterated over linearly, which leads
to few cache misses.

The _bodies are not accessed during iteration. Instead read only data, such as
the mass values are stored with the constraints. The mutable data are the constraint
impulses and the _bodies velocities/positions. The impulses are held inside the
constraint structures. The body velocities/positions are held in compact, temporary
arrays to increase the number of cache hits. Linear and angular velocity are
stored in a single array since multiple arrays lead to multiple misses.
*/

/*
2D Rotation

R = [cos(theta) -sin(theta)]
    [sin(theta) cos(theta) ]

thetaDot = omega

Let q1 = cos(theta), q2 = sin(theta).
R = [q1 -q2]
    [q2  q1]

q1Dot = -thetaDot * q2
q2Dot = thetaDot * q1

q1_new = q1_old - dt * w * q2
q2_new = q2_old + dt * w * q1
then normalize.

This might be faster than computing sin+cos.
However, we can compute sin+cos of the same angle fast.
*/

using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Body;
using Box2D.NetStandard.Dynamics.Contacts;
using Box2D.NetStandard.Dynamics.Joints;
using Box2D.NetStandard.Dynamics.World;
using int32 = System.Int32;
using b2Vec2 = System.Numerics.Vector2;


namespace Box2D.NetStandard.Dynamics {
  public struct Position {
    public Vector2 c;
    public float   a;
  }

  public struct Velocity {
    public Vector2 v;
    public float   w;
  }

  public class Island : IDisposable {
    public ContactListener _listener;

    public Body.Body[]    _bodies;
    public Contact[] _contacts;
    public Joint[]   _joints;

    public Position[] _positions;
    public Velocity[] _velocities;

    public int _bodyCount;
    public int _jointCount;
    public int _contactCount;

    public int _bodyCapacity;
    public int _contactCapacity;
    public int _jointCapacity;

    public int _positionIterationCount;

    public Island(int bodyCapacity, int contactCapacity, int jointCapacity, ContactListener listener) {
      _bodyCapacity    = bodyCapacity;
      _contactCapacity = contactCapacity;
      _jointCapacity   = jointCapacity;
      //__bodyCount = 0;
      //_contactCount = 0;
      //_jointCount = 0;

      _listener = listener;

      _bodies   = new Body.Body[bodyCapacity];
      _contacts = new Contact[contactCapacity];
      _joints   = new Joint[jointCapacity];

      _velocities = new Velocity[_bodyCapacity];
      _positions  = new Position[_bodyCapacity];
    }

    public void Dispose() {
      // Warning: the order should reverse the constructor order.
      _positions  = null;
      _velocities = null;
      _joints     = null;
      _contacts   = null;
      _bodies     = null;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Clear() {
      _bodyCount    = 0;
      _contactCount = 0;
      _jointCount   = 0;
    }

    internal void Solve(Profile profile, in TimeStep step, in Vector2 gravity, bool allowSleep) {
      Stopwatch timer = Stopwatch.StartNew();

      float h = step.dt;
      // Integrate velocities and apply damping.
      for (int i = 0; i < _bodyCount; ++i) {
        Body.Body b = _bodies[i];

        Vector2 c = b._sweep.c;
        float   a = b._sweep.a;
        Vector2 v = b._linearVelocity;
        float   w = b._angularVelocity;

        b._sweep.c0 = b._sweep.c;
        b._sweep.a0 = b._sweep.a;

        if (b._type == BodyType.Dynamic) {
          // Integrate velocities.
          v += h * b._invMass * (b._gravityScale * b._mass * gravity + b._force);
          w += h * b._invI    * b._torque;

          // Apply damping.
          // ODE: dv/dt + c * v = 0
          // Solution: v(t) = v0 * exp(-c * t)
          // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
          // v2 = exp(-c * dt) * v1
          // Pade approximation:
          // v2 = v1 * 1 / (1 + c * dt)
          
          v *= 1.0f / (1.0f + h * b._linearDamping);
          w *= 1.0f / (1.0f + h * b._angularDamping);
        }

        _positions[i].c  = c;
        _positions[i].a  = a;
        _velocities[i].v = v;
        _velocities[i].w = w;
      }

      timer.Restart();

      // Solver data
      SolverData solverData = new SolverData();
      solverData.step       = step;
      solverData.positions  = _positions;
      solverData.velocities = _velocities;

      // Initialize velocity constraints.
      ContactSolverDef contactSolverDef;
      contactSolverDef.step       = step;
      contactSolverDef.contacts   = _contacts;
      contactSolverDef.count      = _contactCount;
      contactSolverDef.positions  = _positions;
      contactSolverDef.velocities = _velocities;
      
      ContactSolver contactSolver = new ContactSolver(contactSolverDef);
      contactSolver.InitializeVelocityConstraints();

      if (step.warmStarting) {
        contactSolver.WarmStart();
      }

      for (int32 i = 0; i < _jointCount; ++i) {
        _joints[i].InitVelocityConstraints(solverData);
      }

      profile.solveInit = timer.ElapsedMilliseconds;

      // Solve velocity constraints
      timer.Reset();
      for (int32 i = 0; i < step.velocityIterations; ++i) {
        for (int32 j = 0; j < _jointCount; ++j) {
          _joints[j].SolveVelocityConstraints(solverData);
        }

        contactSolver.SolveVelocityConstraints();
      }

      // Store impulses for warm starting
      contactSolver.StoreImpulses();
      profile.solveVelocity = timer.ElapsedMilliseconds;

      // Integrate positions
      for (int32 i = 0; i < _bodyCount; ++i) {
        b2Vec2 c = _positions[i].c;
        float  a = _positions[i].a;
        b2Vec2 v = _velocities[i].v;
        float  w = _velocities[i].w;

        // Check for large velocities
        b2Vec2 translation = h * v;
        if (Vector2.Dot(translation, translation) > Settings.MaxTranslationSquared) {
          float ratio = Settings.MaxTranslation / translation.Length();
          v *= ratio;
        }

        float rotation = h * w;
        if (rotation * rotation > Settings.MaxRotationSquared) {
          float ratio = Settings.MaxRotation / MathF.Abs(rotation);
          w *= ratio;
        }

        // Integrate
        c += h * v;
        a += h * w;

        _positions[i].c  = c;
        _positions[i].a  = a;
        _velocities[i].v = v;
        _velocities[i].w = w;
      }

      // Solve position constraints
      timer.Restart();
      bool positionSolved = false;
      for (int32 i = 0; i < step.positionIterations; ++i) {
        bool contactsOkay = contactSolver.SolvePositionConstraints();

        bool jointsOkay = true;
        for (int32 j = 0; j < _jointCount; ++j) {
          bool jointOkay = _joints[j].SolvePositionConstraints(solverData);
          jointsOkay = jointsOkay && jointOkay;
        }

        if (contactsOkay && jointsOkay) {
          // Exit early if the position errors are small.
          positionSolved = true;
          break;
        }
      }

      // Copy state buffers back to the bodies
      for (int32 i = 0; i < _bodyCount; ++i) {
        Body.Body body = _bodies[i];
        body._sweep.c         = _positions[i].c;
        body._sweep.a         = _positions[i].a;
        body._linearVelocity  = _velocities[i].v;
        body._angularVelocity = _velocities[i].w;
        body.SynchronizeTransform();
      }

      profile.solvePosition = timer.ElapsedMilliseconds;

      Report(contactSolver._velocityConstraints);

      if (allowSleep) {
        float minSleepTime = float.MaxValue;

        const float linTolSqr = Settings.LinearSleepTolerance  * Settings.LinearSleepTolerance;
        const float angTolSqr = Settings.AngularSleepTolerance * Settings.AngularSleepTolerance;

        for (int32 i = 0; i < _bodyCount; ++i) {
          Body.Body b = _bodies[i];
          if (b.Type()==BodyType.Static) {
            continue;
          }

          if (!b.HasFlag(BodyFlags.AutoSleep)        ||
              b._angularVelocity * b._angularVelocity     > angTolSqr ||
              Vector2.Dot(b._linearVelocity, b._linearVelocity) > linTolSqr) {
            b._sleepTime = 0.0f;
            minSleepTime  = 0.0f;
          }
          else {
            b._sleepTime += h;
            minSleepTime  =  System.Math.Min(minSleepTime, b._sleepTime);
          }
        }

        if (minSleepTime >= Settings.TimeToSleep && positionSolved) {
          for (int32 i = 0; i < _bodyCount; ++i) {
            Body.Body b = _bodies[i];
            b.SetAwake(false);
          }
        }
      }
    }

    internal void SolveTOI(in TimeStep subStep, int toiIndexA, int toiIndexB) {
      Debug.Assert(toiIndexA < _bodyCount);
      Debug.Assert(toiIndexB < _bodyCount);

      for (int i = 0; i < _bodyCount; i++) {
        Body.Body b = _bodies[i];
        _positions[i].c  = b._sweep.c;
        _positions[i].a  = b._sweep.a;
        _velocities[i].v = b._linearVelocity;
        _velocities[i].w = b._angularVelocity;
      }


      ContactSolverDef contactSolverDef = new ContactSolverDef();
      contactSolverDef.contacts   = _contacts;
      contactSolverDef.count      = _contactCount;
      contactSolverDef.step       = subStep;
      contactSolverDef.positions  = _positions;
      contactSolverDef.velocities = _velocities;

      //ContactSolver contactSolver = new ContactSolver(subStep, _contacts, _contactCount);
      ContactSolver contactSolver = new ContactSolver(contactSolverDef);

      for (int i = 0; i < subStep.positionIterations; ++i) {
        if (contactSolver.SolveTOIPositionConstraints(toiIndexA, toiIndexB)) break;
      }

      // Leap of faith to new safe state
      _bodies[toiIndexA]._sweep.c0 = _positions[toiIndexA].c;
      _bodies[toiIndexA]._sweep.a0 = _positions[toiIndexA].a;
      _bodies[toiIndexB]._sweep.c0 = _positions[toiIndexB].c;
      _bodies[toiIndexB]._sweep.a0 = _positions[toiIndexB].a;

      // No warm starting is needed for TOI events because warm
      // starting impulses were applied in the discrete solver.
      contactSolver.InitializeVelocityConstraints();

      // Solve velocity constraints.
      for (int i = 0; i < subStep.velocityIterations; ++i) {
        contactSolver.SolveVelocityConstraints();
      }

      // Don't store the TOI contact forces for warm starting
      // because they can be quite large.

      float h = subStep.dt;

      // Integrate positions.
      for (int i = 0; i < _bodyCount; ++i) {
        Vector2 c = _positions[i].c;
        float   a = _positions[i].a;
        Vector2 v = _velocities[i].v;
        float   w = _velocities[i].w;

        // Check for large velocities.
        Vector2 translation = h * v;
        if (Vector2.Dot(translation, translation) > Settings.MaxTranslationSquared) {
          float ratio = Settings.MaxTranslation / translation.Length();
          v *= ratio;
        }

        float rotation = h * w;
        if (rotation * rotation > Settings.MaxRotationSquared) {
          float ratio = Settings.MaxRotation / MathF.Abs(rotation);
          w *= ratio;
        }

        // Integrate
        c += h * v;
        a += h * w;

        _positions[i].c  = c;
        _positions[i].a  = a;
        _velocities[i].v = v;
        _velocities[i].w = w;

        // Sync bodies
        Body.Body body = _bodies[i];
        body._sweep.c         = c;
        body._sweep.a         = a;
        body._linearVelocity  = v;
        body._angularVelocity = w;
        body.SynchronizeTransform();
      }

      Report(contactSolver._velocityConstraints);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Add(Body.Body body) {
      Debug.Assert(_bodyCount < _bodyCapacity);
      body._islandIndex     = _bodyCount;
      _bodies[_bodyCount++] = body;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Add(Contact contact) {
      Debug.Assert(_contactCount < _contactCapacity);
      _contacts[_contactCount++] = contact;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Add(Joint joint) {
      Debug.Assert(_jointCount < _jointCapacity);
      _joints[_jointCount++] = joint;
    }

    public void Report(ContactVelocityConstraint[] constraints) {
      if (_listener == null) {
        return;
      }

      for (int i = 0; i < _contactCount; ++i) {
        Contact                   c       = _contacts[i];
        ContactVelocityConstraint cc      = constraints[i];
        ContactImpulse            impulse = new ContactImpulse();
        for (int j = 0; j < cc.pointCount; ++j) {
          impulse.normalImpulses[j]  = cc.points[j].normalImpulse;
          impulse.tangentImpulses[j] = cc.points[j].tangentImpulse;
        }

        _listener.PostSolve(c, impulse);
      }
    }
  }

  public class SolverData {
    internal TimeStep   step;
    internal Position[] positions;
    internal Velocity[] velocities;
  }
}