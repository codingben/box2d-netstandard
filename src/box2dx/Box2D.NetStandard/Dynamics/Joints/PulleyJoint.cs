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


// Pulley:
// length1 = norm(p1 - s1)
// length2 = norm(p2 - s2)
// C0 = (length1 + ratio * length2)_initial
// C = C0 - (length1 + ratio * length2) >= 0
// u1 = (p1 - s1) / norm(p1 - s1)
// u2 = (p2 - s2) / norm(p2 - s2)
// Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
// J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)
//
// Limit:
// C = maxLength - length
// u = (p - s) / norm(p - s)
// Cdot = -dot(u, v + cross(w, r))
// K = invMass + invI * cross(r, u)^2
// 0 <= impulse

using System;
using System.Diagnostics;
using System.Numerics;
using Box2D.NetStandard.Common;
using Math = Box2D.NetStandard.Common.Math;

namespace Box2D.NetStandard.Dynamics.Joints {
  /// <summary>
  /// Pulley joint definition. This requires two ground anchors,
  /// two dynamic body anchor points, max lengths for each side,
  /// and a pulley ratio.
  /// </summary>
  public class PulleyJointDef : JointDef {
    public PulleyJointDef() {
      Type             = JointType.PulleyJoint;
      GroundAnchorA    = new Vector2(-1.0f, 1.0f);
      GroundAnchorB    = new Vector2(1.0f,  1.0f);
      LocalAnchorA     = new Vector2(-1.0f, 0.0f);
      LocalAnchorB     = new Vector2(1.0f,  0.0f);
      LengthA          = 0.0f;
      LengthB          = 0.0f;
      Ratio            = 1.0f;
      CollideConnected = true;
    }

    /// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
    public void Initialize(Body.Body body1,         Body.Body    body2,
      Vector2                   groundAnchor1, Vector2 groundAnchor2,
      Vector2                   anchor1,       Vector2 anchor2,
      float                     ratio) {
      BodyA         = body1;
      BodyB         = body2;
      GroundAnchorA = groundAnchor1;
      GroundAnchorB = groundAnchor2;
      LocalAnchorA  = body1.GetLocalPoint(anchor1);
      LocalAnchorB  = body2.GetLocalPoint(anchor2);
      Vector2 dA = anchor1 - groundAnchor1;
      LengthA = dA.Length();
      Vector2 dB = anchor2 - groundAnchor2;
      LengthB = dB.Length();
      Ratio   = ratio;
      Debug.Assert(ratio > Settings.FLT_EPSILON);
    }

    /// <summary>
    /// The first ground anchor in world coordinates. This point never moves.
    /// </summary>
    public Vector2 GroundAnchorA;

    /// <summary>
    /// The second ground anchor in world coordinates. This point never moves.
    /// </summary>
    public Vector2 GroundAnchorB;

    /// <summary>
    /// The local anchor point relative to body1's origin.
    /// </summary>
    public Vector2 LocalAnchorA;

    /// <summary>
    /// The local anchor point relative to body2's origin.
    /// </summary>
    public Vector2 LocalAnchorB;

    /// <summary>
    /// The a reference length for the segment attached to body1.
    /// </summary>
    public float LengthA;

    /// <summary>
    /// The maximum length of the segment attached to body1.
    /// </summary>
    public float MaxLength1;

    /// <summary>
    /// The a reference length for the segment attached to body2.
    /// </summary>
    public float LengthB;

    /// <summary>
    /// The maximum length of the segment attached to body2.
    /// </summary>
    public float MaxLength2;

    /// <summary>
    /// The pulley ratio, used to simulate a block-and-tackle.
    /// </summary>
    public float Ratio;
  }

  /// <summary>
  /// The pulley joint is connected to two bodies and two fixed ground points.
  /// The pulley supports a ratio such that:
  /// length1 + ratio * length2 <= constant
  /// Yes, the force transmitted is scaled by the ratio.
  /// The pulley also enforces a maximum length limit on both sides. This is
  /// useful to prevent one side of the pulley hitting the top.
  /// </summary>
  public class PulleyJoint : Joint {
    public static readonly float MinPulleyLength = 2.0f;

    private Vector2 _groundAnchorA;
    private Vector2 _groundAnchorB;
    private float   _lengthA;
    private float   _lengthB;

    private Vector2 _localAnchorA;
    private Vector2 _localAnchorB;
    private float   _constant;
    private float   _ratio;
    private float   _impulse;

    private int     _indexA;
    private int     _indexB;
    private Vector2 _uA;
    private Vector2 _uB;
    private Vector2 _rA;
    private Vector2 _rB;
    private Vector2 _localCenterA;
    private Vector2 _localCenterB;
    private float   _invMassA;
    private float   _invMassB;
    private float   _invIA;
    private float   _invIB;
    private float   _mass;


    public override Vector2 Anchor1 {
      get { return _bodyA.GetWorldPoint(_localAnchorA); }
    }

    public override Vector2 Anchor2 {
      get { return _bodyB.GetWorldPoint(_localAnchorB); }
    }

    public override Vector2 GetReactionForce(float inv_dt) {
      Vector2 P = _impulse * _uB;
      return inv_dt * P;
    }

    public override float GetReactionTorque(float inv_dt) {
      return 0.0f;
    }

    /// <summary>
    /// Get the first ground anchor.
    /// </summary>
    public Vector2 GroundAnchorA {
      get { return _groundAnchorA; }
    }

    /// <summary>
    /// Get the second ground anchor.
    /// </summary>
    public Vector2 GroundAnchorB {
      get { return  _groundAnchorB; }
    }

    /// <summary>
    /// Get the current length of the segment attached to body1.
    /// </summary>
    public float LengthA {
      get {
        return _lengthA;
      }
    }

    /// <summary>
    /// Get the current length of the segment attached to body2.
    /// </summary>
    public float LengthB {
      get {
        return _lengthB;
      }
    }

    /// <summary>
    /// Get the pulley ratio.
    /// </summary>
    public float Ratio {
      get { return _ratio; }
    }

    public PulleyJoint(PulleyJointDef def)
      : base(def) {
      _groundAnchorA = def.GroundAnchorA;
      _groundAnchorB = def.GroundAnchorB;
      _localAnchorA  = def.LocalAnchorA;
      _localAnchorB  = def.LocalAnchorB;

      _lengthA = def.LengthA;
      _lengthB = def.LengthB;

      Debug.Assert(def.Ratio != 0.0f);
      _ratio = def.Ratio;

      _constant = def.LengthA + _ratio * def.LengthB;

      _impulse = 0.0f;
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

      Vector2 cA =  data.positions[_indexA].c;
      float  aA =  data.positions[_indexA].a;
      Vector2 vA = data.velocities[_indexA].v;
      float  wA = data.velocities[_indexA].w;

      Vector2 cB =  data.positions[_indexB].c;
      float  aB =  data.positions[_indexB].a;
      Vector2 vB = data.velocities[_indexB].v;
      float  wB = data.velocities[_indexB].w;

      Rot qA = new Rot(aA), qB = new Rot(aB);

      _rA = Math.Mul(qA, _localAnchorA - _localCenterA);
      _rB = Math.Mul(qB, _localAnchorB - _localCenterB);

      // Get the pulley axes.
      _uA = cA + _rA - _groundAnchorA;
      _uB = cB + _rB - _groundAnchorB;

      float lengthA = _uA.Length();
      float lengthB = _uB.Length();

      if (lengthA > 10.0f * Settings.LinearSlop) {
        _uA *= 1.0f / lengthA;
      }
      else {
        _uA = Vector2.Zero;
      }

      if (lengthB > 10.0f * Settings.LinearSlop) {
        _uB *= 1.0f / lengthB;
      }
      else {
        _uB = Vector2.Zero;
      }

      // Compute effective mass.
      float ruA = Vectex.Cross(_rA, _uA);
      float ruB = Vectex.Cross(_rB, _uB);

      float mA = _invMassA + _invIA * ruA * ruA;
      float mB = _invMassB + _invIB * ruB * ruB;

      _mass = mA + _ratio * _ratio * mB;

      if (_mass > 0.0f) {
        _mass = 1.0f / _mass;
      }

      if (data.step.warmStarting) {
        // Scale impulses to support variable time steps.
        _impulse *= data.step.dtRatio;

        // Warm starting.
        Vector2 PA = -(_impulse)          * _uA;
        Vector2 PB = (-_ratio * _impulse) * _uB;

        vA += _invMassA * PA;
        wA += _invIA    * Vectex.Cross(_rA, PA);
        vB += _invMassB * PB;
        wB += _invIB    * Vectex.Cross(_rB, PB);
      }
      else {
        _impulse = 0.0f;
      }

      data.velocities[_indexA].v = vA;
      data.velocities[_indexA].w = wA;
      data.velocities[_indexB].v = vB;
      data.velocities[_indexB].w = wB;
    }

    internal override void SolveVelocityConstraints(in SolverData data) {
      Vector2 vA = data.velocities[_indexA].v;
      float  wA = data.velocities[_indexA].w;
      Vector2 vB = data.velocities[_indexB].v;
      float  wB = data.velocities[_indexB].w;

      Vector2 vpA = vA + Vectex.Cross(wA, _rA);
      Vector2 vpB = vB + Vectex.Cross(wB, _rB);

      float Cdot    = -Vector2.Dot(_uA, vpA) - _ratio * Vector2.Dot(_uB, vpB);
      float impulse = -_mass * Cdot;
      _impulse += impulse;

      Vector2 PA = -impulse           * _uA;
      Vector2 PB = -_ratio * impulse * _uB;
      vA += _invMassA * PA;
      wA += _invIA    * Vectex.Cross(_rA, PA);
      vB += _invMassB * PB;
      wB += _invIB    * Vectex.Cross(_rB, PB);

      data.velocities[_indexA].v = vA;
      data.velocities[_indexA].w = wA;
      data.velocities[_indexB].v = vB;
      data.velocities[_indexB].w = wB;
    }

    internal override bool SolvePositionConstraints(in SolverData data) {
      Vector2 cA = data.positions[_indexA].c;
      float  aA = data.positions[_indexA].a;
      Vector2 cB = data.positions[_indexB].c;
      float  aB = data.positions[_indexB].a;

      Rot qA= new Rot(aA), qB= new Rot(aB);

      Vector2 rA = Math.Mul(qA, _localAnchorA - _localCenterA);
      Vector2 rB = Math.Mul(qB, _localAnchorB - _localCenterB);

      // Get the pulley axes.
      Vector2 uA = cA + rA - _groundAnchorA;
      Vector2 uB = cB + rB - _groundAnchorB;

      float lengthA = uA.Length();
      float lengthB = uB.Length();

      if (lengthA > 10.0f * Settings.LinearSlop)
      {
        uA *= 1.0f / lengthA;
      }
      else
      {
        uA = Vector2.Zero;
      }

      if (lengthB > 10.0f * Settings.LinearSlop)
      {
        uB *= 1.0f / lengthB;
      }
      else
      {
        uB = Vector2.Zero;
      }

      // Compute effective mass.
      float ruA = Vectex.Cross(rA, uA);
      float ruB = Vectex.Cross(rB, uB);
                          
      float mA = _invMassA + _invIA * ruA * ruA;
      float mB = _invMassB + _invIB * ruB * ruB;

      float mass = mA + _ratio * _ratio * mB;

      if (mass > 0.0f)
      {
        mass = 1.0f / mass;
      }

      float C           = _constant - lengthA - _ratio * lengthB;
      float linearError = MathF.Abs(C);

      float impulse = -mass * C;

      Vector2 PA = -impulse           * uA;
      Vector2 PB = -_ratio * impulse * uB;

      cA += _invMassA * PA;
      aA += _invIA    * Vectex.Cross(rA, PA);
      cB += _invMassB * PB;
      aB += _invIB    * Vectex.Cross(rB, PB);

      data.positions[_indexA].c = cA;
      data.positions[_indexA].a = aA;
      data.positions[_indexB].c = cB;
      data.positions[_indexB].a = aB;

      return linearError < Settings.LinearSlop;
    }
  }
}