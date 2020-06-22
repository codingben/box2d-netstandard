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


// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k * 

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.World;
using Math = Box2D.NetStandard.Common.Math;

namespace Box2D.NetStandard.Dynamics.Joints.Distance {
  /// <summary>
  /// A distance joint constrains two points on two bodies
  /// to remain at a fixed distance from each other. You can view
  /// this as a massless, rigid rod.
  /// </summary>
  public class DistanceJoint : Joint {
    private Vector2 _localAnchorA;
    private Vector2 _localAnchorB;
    private Vector2 _u;
    private float   _frequencyHz;
    private float   _dampingRatio;
    private float   _gamma;
    private float   _bias;
    private float   _impulse;
    private float   _mass; // effective mass for the constraint.
    private float   _length;
    private int _indexA;
    private int _indexB;
    private Vector2 _localCenterA;
    private Vector2 _localCenterB;
    private float _invMassA;
    private float _invMassB;
    private float _invIA;
    private float _invIB;
    private Vector2 _rA;
    private Vector2 _rB;

    public override Vector2 GetAnchorA => _bodyA.GetWorldPoint(_localAnchorA);

    public override Vector2 GetAnchorB => _bodyB.GetWorldPoint(_localAnchorB);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override Vector2 GetReactionForce(float inv_dt) => (inv_dt * _impulse) * _u;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override float GetReactionTorque(float inv_dt) => 0.0f;

    public DistanceJoint(DistanceJointDef def)
      : base(def) {
      _localAnchorA = def.localAnchorA;
      _localAnchorB = def.localAnchorB;
      _length       = def.length;
      _frequencyHz  = def.frequencyHz;
      _dampingRatio = def.dampingRatio;
      _impulse      = 0.0f;
      _gamma        = 0.0f;
      _bias         = 0.0f;
    }

    internal override void InitVelocityConstraints(in SolverData data) {
      _indexA       = _bodyA._islandIndex;
      _indexB       = _bodyB._islandIndex;
      _localCenterA = _bodyA._sweep.localCenter;
      _localCenterB = _bodyB._sweep.localCenter;
      _invMassA     = _bodyA._invMass;
      _invMassB     = _bodyB._invMass;
      _invIA        = _bodyA._invI;
      _invIB        = _bodyB._invI;

      Vector2 cA = data.positions[ _indexA].c;
      float  aA = data.positions[ _indexA].a;
      Vector2 vA = data.velocities[_indexA].v;
      float  wA = data.velocities[_indexA].w;

      Vector2 cB = data.positions[ _indexB].c;
      float  aB = data.positions[ _indexB].a;
      Vector2 vB = data.velocities[_indexB].v;
      float  wB = data.velocities[_indexB].w;

      Rot qA = new Rot(aA), qB = new Rot(aB);

      _rA = Math.Mul(qA, _localAnchorA - _localCenterA);
      _rB = Math.Mul(qB, _localAnchorB - _localCenterB);
      _u  = cB + _rB - cA - _rA;

      // Handle singularity.
      float length = _u.Length();
      if (length > Settings.LinearSlop) {
        _u *= 1.0f / length;
      }
      else {
        _u=Vector2.Zero;
      }

      float crAu    = Vectex.Cross(_rA, _u);
      float crBu    = Vectex.Cross(_rB, _u);
      float invMass = _invMassA + _invIA * crAu * crAu + _invMassB + _invIB * crBu * crBu;

      // Compute the effective mass matrix.
      _mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

      if (_frequencyHz > 0.0f) {
        float C = length - _length;

        // Frequency
        float omega = 2.0f * MathF.PI * _frequencyHz;

        // Damping coefficient
        float d = 2.0f * _mass * _dampingRatio * omega;

        // Spring stiffness
        float k = _mass * omega * omega;

        // magic formulas
        float h = data.step.dt;

        // gamma = 1 / (h * (d + h * k)), the extra factor of h in the denominator is since the lambda is an impulse, not a force
        _gamma = h * (d + h * k);
        _gamma = _gamma != 0.0f ? 1.0f / _gamma : 0.0f;
        _bias  = C * h * k * _gamma;

        invMass += _gamma;
        _mass  =  invMass != 0.0f ? 1.0f / invMass : 0.0f;
      }
      else {
        _gamma = 0.0f;
        _bias  = 0.0f;
      }

      if (data.step.warmStarting) {
        // Scale the impulse to support a variable time step.
        _impulse *= data.step.dtRatio;

        Vector2 P = _impulse * _u;
        vA -= _invMassA * P;
        wA -= _invIA    * Vectex.Cross(_rA, P);
        vB += _invMassB * P;
        wB += _invIB    * Vectex.Cross(_rB, P);
      }
      else {
        _impulse = 0.0f;
      }

      data.velocities[_indexA].v = vA;
      data.velocities[_indexA].w = wA;
      data.velocities[_indexB].v = vB;
      data.velocities[_indexB].w = wB;
    }

    internal override bool SolvePositionConstraints(in SolverData data) {
      if (_frequencyHz > 0.0f) {
        //There is no position correction for soft distance constraints.
        return true;
      }

      Body b1 = _bodyA;
      Body b2 = _bodyB;

      Vector2 r1 = Math.Mul(b1.GetTransform().q, _localAnchorA - b1.GetLocalCenter());
      Vector2 r2 = Math.Mul(b2.GetTransform().q, _localAnchorB - b2.GetLocalCenter());

      Vector2 d = b2._sweep.c + r2 - b1._sweep.c - r1;

      float length = d.Length();
      d = Vector2.Normalize(d);
      float C = length - _length;
      C = System.Math.Clamp(C, -Settings.MaxLinearCorrection, Settings.MaxLinearCorrection);

      float impulse = -_mass * C;
      _u = d;
      Vector2 P = impulse * _u;

      b1._sweep.c -= b1._invMass * P;
      b1._sweep.a -= b1._invI    * Vectex.Cross(r1, P);
      b2._sweep.c += b2._invMass * P;
      b2._sweep.a += b2._invI    * Vectex.Cross(r2, P);

      b1.SynchronizeTransform();
      b2.SynchronizeTransform();

      return System.Math.Abs(C) < Settings.LinearSlop;
    }

    internal override void SolveVelocityConstraints(in SolverData data) {
      Vector2 vA = data.velocities[_indexA].v;
      float  wA = data.velocities[_indexA].w;
      Vector2 vB = data.velocities[_indexB].v;
      float  wB = data.velocities[_indexB].w;

      // Cdot = dot(u, v + cross(w, r))
      Vector2 vpA  = vA + Vectex.Cross(wA, _rA);
      Vector2 vpB  = vB + Vectex.Cross(wB, _rB);
      float  Cdot = Vector2.Dot(_u, vpB - vpA);

      float impulse = -_mass * (Cdot + _bias + _gamma * _impulse);
      _impulse += impulse;

      Vector2 P = impulse * _u;
      vA -= _invMassA * P;
      wA -= _invIA    * Vectex.Cross(_rA, P);
      vB += _invMassB * P;
      wB += _invIB    * Vectex.Cross(_rB, P);

      data.velocities[_indexA].v = vA;
      data.velocities[_indexA].w = wA;
      data.velocities[_indexB].v = vB;
      data.velocities[_indexB].w = wB;
    }
  }
}