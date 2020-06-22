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


// Gear Joint:
// C0 = (coordinate1 + ratio * coordinate2)_initial
// C = C0 - (cordinate1 + ratio * coordinate2) = 0
// Cdot = -(Cdot1 + ratio * Cdot2)
// J = -[J1 ratio * J2]
// K = J * invM * JT
//   = J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
//
// Revolute:
// coordinate = rotation
// Cdot = angularVelocity
// J = [0 0 1]
// K = J * invM * JT = invI
//
// Prismatic:
// coordinate = dot(p - pg, ug)
// Cdot = dot(v + cross(w, r), ug)
// J = [ug cross(r, ug)]
// K = J * invM * JT = invMass + invI * cross(r, ug)^2

using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Joints.Prismatic;
using Box2D.NetStandard.Dynamics.Joints.Revolute;
using Box2D.NetStandard.Dynamics.World;

namespace Box2D.NetStandard.Dynamics.Joints.Gear {
  /// <summary>
  /// A gear joint is used to connect two joints together. Either joint
  /// can be a revolute or prismatic joint. You specify a gear ratio
  /// to bind the motions together:
  /// coordinate1 + ratio * coordinate2 = constant
  /// The ratio can be negative or positive. If one joint is a revolute joint
  /// and the other joint is a prismatic joint, then the ratio will have units
  /// of length or units of 1/length.
  /// @warning The revolute and prismatic joints must be attached to
  /// fixed bodies (which must be body1 on those joints).
  /// </summary>
  public class GearJoint : Joint {
    private Joint     _joint1;
    private Joint     _joint2;
    private JointType _typeA;
    private JointType _typeB;
    private Body      _bodyC;
    private Vector2   _localAnchorC;
    private Vector2   _localAnchorA;
    private float     _referenceAngleA;
    private Vector2   _localAxisC;
    private Body      _bodyD;
    private Vector2   _localAnchorD;
    private Vector2   _localAnchorB;
    private float     _referenceAngleB;
    private Vector2   _localAxisD;
    private float     _ratio;
    private float     _constant;
    private float     _impulse;
    private int       _indexA;
    private int       _indexB;
    private int       _indexC;
    private int       _indexD;
    private Vector2   _lcA;
    private Vector2   _lcB;
    private Vector2   _lcC;
    private Vector2   _lcD;
    private float     _mA;
    private float     _mB;
    private float     _mC;
    private float     _mD;
    private float     _iA;
    private float     _iB;
    private float     _iC;
    private float     _iD;
    private float     _mass;
    private Vector2   _JvAC;
    private Vector2   _JvBD;
    private float     _JwA;
    private float     _JwB;
    private float     _JwC;
    private float     _JwD;


    public override Vector2 GetAnchorA => _bodyA.GetWorldPoint(_localAnchorA);
    public override Vector2 GetAnchorB => _bodyB.GetWorldPoint(_localAnchorB);
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override Vector2 GetReactionForce(float inv_dt) => _impulse * _JvAC;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override float GetReactionTorque(float inv_dt) => inv_dt * _impulse * _JwA;

    /// <summary>
    /// Get the gear ratio.
    /// </summary>
    public float Ratio {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => _ratio;
    }

    public GearJoint(GearJointDef def)
      : base(def) {
      _joint1 = def.Joint1;
      _joint2 = def.Joint2;

      _typeA = _joint1.Type;
      _typeB = _joint2.Type;

      //Debug.Assert(_typeA == JointType.RevoluteJoint || _typeA == JointType.PrismaticJoint);
      //Debug.Assert(_typeB == JointType.RevoluteJoint || _typeB == JointType.PrismaticJoint);

      float coordinateA, coordinateB;

      // TODO_ERIN there might be some problem with the joint edges in b2Joint.

      _bodyC = _joint1.GetBodyA();
      _bodyA = _joint1.GetBodyB();

      // Get geometry of joint1
      Transform xfA = _bodyA._xf;
      float     aA  = _bodyA._sweep.a;
      Transform xfC = _bodyC._xf;
      float     aC  = _bodyC._sweep.a;

      if (_typeA == JointType.RevoluteJoint) {
        RevoluteJoint revolute = (RevoluteJoint) def.Joint1;
        _localAnchorC    = revolute._localAnchorA;
        _localAnchorA    = revolute._localAnchorB;
        _referenceAngleA = revolute._referenceAngle;
        _localAxisC      = Vector2.Zero;

        coordinateA = aA - aC - _referenceAngleA;
      }
      else {
        PrismaticJoint prismatic = (PrismaticJoint) def.Joint1;
        _localAnchorC    = prismatic._localAnchorA;
        _localAnchorA    = prismatic._localAnchorB;
        _referenceAngleA = prismatic._referenceAngle;
        _localAxisC      = prismatic._localXAxisA;

        Vector2 pC = _localAnchorC;
        Vector2 pA = Math.MulT(xfC.q, Math.Mul(xfA.q, _localAnchorA) + (xfA.p - xfC.p));
        coordinateA = Vector2.Dot(pA - pC, _localAxisC);
      }

      _bodyD = _joint2.GetBodyA();
      _bodyB = _joint2.GetBodyB();

      // Get geometry of joint2
      Transform xfB = _bodyB._xf;
      float     aB  = _bodyB._sweep.a;
      Transform xfD = _bodyD._xf;
      float     aD  = _bodyD._sweep.a;

      if (_typeB == JointType.RevoluteJoint) {
        RevoluteJoint revolute = (RevoluteJoint) def.Joint2;
        _localAnchorD    = revolute._localAnchorA;
        _localAnchorB    = revolute._localAnchorB;
        _referenceAngleB = revolute._referenceAngle;
        _localAxisD      = Vector2.Zero;

        coordinateB = aB - aD - _referenceAngleB;
      }
      else {
        PrismaticJoint prismatic = (PrismaticJoint) def.Joint2;
        _localAnchorD    = prismatic._localAnchorA;
        _localAnchorB    = prismatic._localAnchorB;
        _referenceAngleB = prismatic._referenceAngle;
        _localAxisD      = prismatic._localXAxisA;

        Vector2 pD = _localAnchorD;
        Vector2 pB = Math.MulT(xfD.q, Math.Mul(xfB.q, _localAnchorB) + (xfB.p - xfD.p));
        coordinateB = Vector2.Dot(pB - pD, _localAxisD);
      }

      _ratio = def.Ratio;

      _constant = coordinateA + _ratio * coordinateB;

      _impulse = 0.0f;
    }

    internal override void InitVelocityConstraints(in SolverData data) {
      _indexA = _bodyA._islandIndex;
      _indexB = _bodyB._islandIndex;
      _indexC = _bodyC._islandIndex;
      _indexD = _bodyD._islandIndex;
      _lcA    = _bodyA._sweep.localCenter;
      _lcB    = _bodyB._sweep.localCenter;
      _lcC    = _bodyC._sweep.localCenter;
      _lcD    = _bodyD._sweep.localCenter;
      _mA     = _bodyA._invMass;
      _mB     = _bodyB._invMass;
      _mC     = _bodyC._invMass;
      _mD     = _bodyD._invMass;
      _iA     = _bodyA._invI;
      _iB     = _bodyB._invI;
      _iC     = _bodyC._invI;
      _iD     = _bodyD._invI;

      float   aA = data.positions[_indexA].a;
      Vector2 vA = data.velocities[_indexA].v;
      float   wA = data.velocities[_indexA].w;

      float   aB = data.positions[_indexB].a;
      Vector2 vB = data.velocities[_indexB].v;
      float   wB = data.velocities[_indexB].w;

      float   aC = data.positions[_indexC].a;
      Vector2 vC = data.velocities[_indexC].v;
      float   wC = data.velocities[_indexC].w;

      float   aD = data.positions[_indexD].a;
      Vector2 vD = data.velocities[_indexD].v;
      float   wD = data.velocities[_indexD].w;

      Rot qA = new Rot(aA), qB = new Rot(aB), qC = new Rot(aC), qD = new Rot(aD);

      _mass = 0.0f;

      if (_typeA == JointType.RevoluteJoint) {
        _JvAC =  Vector2.Zero;
        _JwA  =  1.0f;
        _JwC  =  1.0f;
        _mass += _iA + _iC;
      }
      else {
        Vector2 u  = Math.Mul(qC, _localAxisC);
        Vector2 rC = Math.Mul(qC, _localAnchorC - _lcC);
        Vector2 rA = Math.Mul(qA, _localAnchorA - _lcA);
        _JvAC =  u;
        _JwC  =  Vectex.Cross(rC, u);
        _JwA  =  Vectex.Cross(rA, u);
        _mass += _mC + _mA + _iC * _JwC * _JwC + _iA * _JwA * _JwA;
      }

      if (_typeB == JointType.RevoluteJoint) {
        _JvBD =  Vector2.Zero;
        _JwB  =  _ratio;
        _JwD  =  _ratio;
        _mass += _ratio * _ratio * (_iB + _iD);
      }
      else {
        Vector2 u  = Math.Mul(qD, _localAxisD);
        Vector2 rD = Math.Mul(qD, _localAnchorD - _lcD);
        Vector2 rB = Math.Mul(qB, _localAnchorB - _lcB);
        _JvBD =  _ratio * u;
        _JwD  =  _ratio * Vectex.Cross(rD, u);
        _JwB  =  _ratio * Vectex.Cross(rB, u);
        _mass += _ratio * _ratio * (_mD + _mB) + _iD * _JwD * _JwD + _iB * _JwB * _JwB;
      }

      // Compute effective mass.
      _mass = _mass > 0.0f ? 1.0f / _mass : 0.0f;

      if (data.step.warmStarting) {
        vA += _mA * _impulse * _JvAC;
        wA += _iA * _impulse * _JwA;
        vB += _mB * _impulse * _JvBD;
        wB += _iB * _impulse * _JwB;
        vC -= _mC * _impulse * _JvAC;
        wC -= _iC * _impulse * _JwC;
        vD -= _mD * _impulse * _JvBD;
        wD -= _iD * _impulse * _JwD;
      }
      else {
        _impulse = 0.0f;
      }

      data.velocities[_indexA].v = vA;
      data.velocities[_indexA].w = wA;
      data.velocities[_indexB].v = vB;
      data.velocities[_indexB].w = wB;
      data.velocities[_indexC].v = vC;
      data.velocities[_indexC].w = wC;
      data.velocities[_indexD].v = vD;
      data.velocities[_indexD].w = wD;
    }

    internal override void SolveVelocityConstraints(in SolverData data) {
      Vector2 vA = data.velocities[_indexA].v;
      float   wA = data.velocities[_indexA].w;
      Vector2 vB = data.velocities[_indexB].v;
      float   wB = data.velocities[_indexB].w;
      Vector2 vC = data.velocities[_indexC].v;
      float   wC = data.velocities[_indexC].w;
      Vector2 vD = data.velocities[_indexD].v;
      float   wD = data.velocities[_indexD].w;

      float Cdot = Vector2.Dot(_JvAC, vA - vC) + Vector2.Dot(_JvBD, vB - vD);
      Cdot += (_JwA * wA                       - _JwC * wC) + (_JwB * wB - _JwD * wD);

      float impulse = -_mass * Cdot;
      _impulse += impulse;

      vA += _mA * impulse * _JvAC;
      wA += _iA * impulse * _JwA;
      vB += _mB * impulse * _JvBD;
      wB += _iB * impulse * _JwB;
      vC -= _mC * impulse * _JvAC;
      wC -= _iC * impulse * _JwC;
      vD -= _mD * impulse * _JvBD;
      wD -= _iD * impulse * _JwD;

      data.velocities[_indexA].v = vA;
      data.velocities[_indexA].w = wA;
      data.velocities[_indexB].v = vB;
      data.velocities[_indexB].w = wB;
      data.velocities[_indexC].v = vC;
      data.velocities[_indexC].w = wC;
      data.velocities[_indexD].v = vD;
      data.velocities[_indexD].w = wD;
    }

    internal override bool SolvePositionConstraints(in SolverData data) {
      Vector2 cA = data.positions[_indexA].c;
      float  aA = data.positions[_indexA].a;
      Vector2 cB = data.positions[_indexB].c;
      float  aB = data.positions[_indexB].a;
      Vector2 cC = data.positions[_indexC].c;
      float  aC = data.positions[_indexC].a;
      Vector2 cD = data.positions[_indexD].c;
      float  aD = data.positions[_indexD].a;

      Rot qA=new Rot(aA), qB=new Rot(aB), qC=new Rot(aC), qD=new Rot(aD);

      float linearError = 0.0f;

      float coordinateA, coordinateB;

      Vector2 JvAC, JvBD;
      float  JwA,  JwB, JwC, JwD;
      float  mass = 0.0f;

      if (_typeA == JointType.RevoluteJoint) {
        JvAC=Vector2.Zero;
        JwA  =  1.0f;
        JwC  =  1.0f;
        mass += _iA + _iC;

        coordinateA = aA - aC - _referenceAngleA;
      }
      else {
        Vector2 u  = Math.Mul(qC, _localAxisC);
        Vector2 rC = Math.Mul(qC, _localAnchorC - _lcC);
        Vector2 rA = Math.Mul(qA, _localAnchorA - _lcA);
        JvAC =  u;
        JwC  =  Vectex.Cross(rC, u);
        JwA  =  Vectex.Cross(rA, u);
        mass += _mC + _mA + _iC * JwC * JwC + _iA * JwA * JwA;

        Vector2 pC = _localAnchorC - _lcC;
        Vector2 pA = Math.MulT(qC, rA + (cA - cC));
        coordinateA = Vector2.Dot(pA - pC, _localAxisC);
      }

      if (_typeB == JointType.RevoluteJoint) {
        JvBD = Vector2.Zero;
        JwB  =  _ratio;
        JwD  =  _ratio;
        mass += _ratio * _ratio * (_iB + _iD);

        coordinateB = aB - aD - _referenceAngleB;
      }
      else {
        Vector2 u  = Math.Mul(qD, _localAxisD);
        Vector2 rD = Math.Mul(qD, _localAnchorD - _lcD);
        Vector2 rB = Math.Mul(qB, _localAnchorB - _lcB);
        JvBD =  _ratio * u;
        JwD  =  _ratio * Vectex.Cross(rD, u);
        JwB  =  _ratio * Vectex.Cross(rB, u);
        mass += _ratio * _ratio * (_mD + _mB) + _iD * JwD * JwD + _iB * JwB * JwB;

        Vector2 pD = _localAnchorD - _lcD;
        Vector2 pB = Math.MulT(qD, rB + (cB - cD));
        coordinateB = Vector2.Dot(pB - pD, _localAxisD);
      }

      float C = (coordinateA + _ratio * coordinateB) - _constant;

      float impulse = 0.0f;
      if (mass > 0.0f) {
        impulse = -C / mass;
      }

      cA += _mA * impulse * JvAC;
      aA += _iA * impulse * JwA;
      cB += _mB * impulse * JvBD;
      aB += _iB * impulse * JwB;
      cC -= _mC * impulse * JvAC;
      aC -= _iC * impulse * JwC;
      cD -= _mD * impulse * JvBD;
      aD -= _iD * impulse * JwD;

      data.positions[_indexA].c = cA;
      data.positions[_indexA].a = aA;
      data.positions[_indexB].c = cB;
      data.positions[_indexB].a = aB;
      data.positions[_indexC].c = cC;
      data.positions[_indexC].a = aC;
      data.positions[_indexD].c = cD;
      data.positions[_indexD].a = aD;

      // TODO_ERIN not implemented
      return linearError < Settings.LinearSlop;
    }
  }
}