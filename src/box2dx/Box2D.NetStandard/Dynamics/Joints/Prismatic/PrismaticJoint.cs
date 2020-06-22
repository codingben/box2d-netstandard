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


// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(perp, d)
// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// Angular constraint
// C = a2 - a1 + a_initial
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -s1 a s2]
//     [0  -1  0  1]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)


// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

// Block Solver
// We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
// when the mass has poor distribution (leading to large torques about the joint anchor points).
//
// The Jacobian has 3 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//     [-vT -a1 vT a2] // limit
//
// u = perp
// v = axis
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

// M * (v2 - v1) = JT * df
// J * v2 = bias
//
// v2 = v1 + invM * JT * df
// J * (v1 + invM * JT * df) = bias
// K * df = bias - J * v1 = -Cdot
// K = J * invM * JT
// Cdot = J * v1 - bias
//
// Now solve for f2.
// df = f2 - f1
// K * (f2 - f1) = -Cdot
// f2 = invK * (-Cdot) + f1
//
// Clamp accumulated limit impulse.
// lower: f2(3) = max(f2(3), 0)
// upper: f2(3) = min(f2(3), 0)
//
// Solve for correct f2(1:2)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
//                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
//
// Now compute impulse to be applied:
// df = f2 - f1

using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.World;
using Math = Box2D.NetStandard.Common.Math;

namespace Box2D.NetStandard.Dynamics.Joints.Prismatic {
  /// <summary>
  /// A prismatic joint. This joint provides one degree of freedom: translation
  /// along an axis fixed in body1. Relative rotation is prevented. You can
  /// use a joint limit to restrict the range of motion and a joint motor to
  /// drive the motion or to model joint friction.
  /// </summary>
  public class PrismaticJoint : Joint {
    internal Vector2 _localAnchorA;
    internal Vector2 _localAnchorB;
    internal Vector2 _localXAxisA;
    private Vector2 _localYAxisA;
    internal float   _referenceAngle;

    private Vector2 _axis, _perp;
    private float   _motorImpulse;
    private float   _lowerTranslation;
    private float   _upperTranslation;
    private float   _maxMotorForce;
    private float   _motorSpeed;
    private bool    _enableLimit;
    private bool    _enableMotor;
    private float   _axialMass;
    private float   _lowerImpulse;
    private float   _upperImpulse;
    private Vector2 _impulse;
    private int     _indexA;
    private int     _indexB;
    private Vector2 _localCenterA;
    private Vector2 _localCenterB;
    private float   _invMassA;
    private float   _invMassB;
    private float   _invIA;
    private float   _invIB;
    private float   _a1;
    private float   _a2;
    private float   _s1;
    private float   _s2;
    private float   _translation;
    private Matrix3x2   _K;

    public override Vector2 GetAnchorA => _bodyA.GetWorldPoint(_localAnchorA);

    public override Vector2 GetAnchorB => _bodyB.GetWorldPoint(_localAnchorB);

    public override Vector2 GetReactionForce(float inv_dt) {
      return inv_dt * (_impulse.X * _perp + (_motorImpulse + _lowerImpulse + _upperImpulse) * _axis);
    }

    public override float GetReactionTorque(float inv_dt) {
      return inv_dt * _impulse.Y;
    }


    /// <summary>
    /// Get the current joint translation, usually in meters.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetJointTranslation(){
       
        Body b1 = _bodyA;
        Body b2 = _bodyB;

        Vector2 p1   = b1.GetWorldPoint(_localAnchorA);
        Vector2 p2   = b2.GetWorldPoint(_localAnchorB);
        Vector2 d    = p2 - p1;
        Vector2 axis = b1.GetWorldVector(_localXAxisA);

        return Vector2.Dot(d, axis);

  }

    /// <summary>
    /// Get the current joint translation speed, usually in meters per second.
    /// </summary>
    public float JointSpeed( ) {
        Body b1 = _bodyA;
        Body b2 = _bodyB;

        Vector2 r1   = Math.Mul(b1.GetTransform().q, _localAnchorA - b1.GetLocalCenter());
        Vector2 r2   = Math.Mul(b2.GetTransform().q, _localAnchorB - b2.GetLocalCenter());
        Vector2 p1   = b1._sweep.c + r1;
        Vector2 p2   = b2._sweep.c + r2;
        Vector2 d    = p2          - p1;
        Vector2 axis = b1.GetWorldVector(_localXAxisA);

        Vector2 v1 = b1._linearVelocity;
        Vector2 v2 = b2._linearVelocity;
        float   w1 = b1._angularVelocity;
        float   w2 = b2._angularVelocity;

        return Vector2.Dot(d,    Vectex.Cross(w1, axis)) +
                      Vector2.Dot(axis, v2 + Vectex.Cross(w2, r2) - v1 - Vectex.Cross(w1, r1));
       
      
    }

    /// <summary>
    /// Is the joint limit enabled?
    /// </summary>
    public bool IsLimitEnabled {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => _enableLimit;
    }

    /// <summary>
    /// Enable/disable the joint limit.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void EnableLimit(bool flag) {
      _bodyA.SetAwake(true);
      _bodyB.SetAwake(true);
      _enableLimit = flag;
    }

    /// <summary>
    /// Get the lower joint limit, usually in meters.
    /// </summary>
    public float LowerLimit {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => _lowerTranslation;
    }

    /// <summary>
    /// Get the upper joint limit, usually in meters.
    /// </summary>
    public float UpperLimit {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => _upperTranslation;
    }

    /// <summary>
    /// Set the joint limits, usually in meters.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetLimits(float lower, float upper) {
      Debug.Assert(lower <= upper);
      _bodyA.SetAwake(true);
      _bodyB.SetAwake(true);
      _lowerTranslation = lower;
      _upperTranslation = upper;
    }

    /// <summary>
    /// Is the joint motor enabled?
    /// </summary>
    public bool IsMotorEnabled {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => _enableMotor;
    }

    /// <summary>
    /// Enable/disable the joint motor.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void EnableMotor(bool flag) {
      _bodyA.SetAwake(true);
      _bodyB.SetAwake(true);
      _enableMotor = flag;
    }

    /// <summary>
    /// Get\Set the motor speed, usually in meters per second.
    /// </summary>
    public float MotorSpeed {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => _motorSpeed;
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      set {
        _bodyA.SetAwake(true);
        _bodyB.SetAwake(true);
        _motorSpeed = value;
      }
    }

    /// <summary>
    /// Set the maximum motor force, usually in N.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetMaxMotorForce(float force) {
      _bodyA.SetAwake(true);
      _bodyB.SetAwake(true);
      _maxMotorForce = Settings.FORCE_SCALE(1.0f) * force;
    }

    /// <summary>
    /// Get the current motor force, usually in N.
    /// </summary>
    public float MotorForce {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => _motorImpulse;
    }

    public PrismaticJoint(PrismaticJointDef def)
      : base(def) {
      _localAnchorA = def.localAnchorA;
      _localAnchorB = def.localAnchorB;
      _localXAxisA  = Vector2.Normalize(def.localAxisA);
      _localYAxisA  = Vectex.Cross(1.0f, _localXAxisA);
      _referenceAngle     = def.referenceAngle;

      _impulse=Vector2.Zero;
      _axialMass    = 0.0f;
      _motorImpulse = 0.0f;
      _lowerImpulse = 0.0f;
      _upperImpulse = 0.0f;

      _lowerTranslation = def.lowerTranslation;
      _upperTranslation = def.upperTranslation;

      _maxMotorForce = def.maxMotorForce;
      _motorSpeed    = def.motorSpeed;
      _enableLimit   = def.enableLimit;
      _enableMotor   = def.enableMotor;

      _axis = Vector2.Zero;
      _perp = Vector2.Zero;
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

      Vector2 cA = data.positions[_indexA].c;
      float   aA = data.positions[_indexA].a;
      Vector2 vA = data.velocities[_indexA].v;
      float   wA = data.velocities[_indexA].w;

      Vector2 cB = data.positions[_indexB].c;
      float   aB = data.positions[_indexB].a;
      Vector2 vB = data.velocities[_indexB].v;
      float   wB = data.velocities[_indexB].w;

      Rot qA = new Rot(aA), qB = new Rot(aB);

      // Compute the effective masses.
      Vector2 rA = Math.Mul(qA, _localAnchorA - _localCenterA);
      Vector2 rB = Math.Mul(qB, _localAnchorB - _localCenterB);
      Vector2 d  = (cB                        - cA) + rB - rA;

      float mA = _invMassA, mB = _invMassB;
      float iA = _invIA,    iB = _invIB;

      // Compute motor Jacobian and effective mass.
      {
        _axis = Math.Mul(qA, _localXAxisA);
        _a1   = Vectex.Cross(d + rA, _axis);
        _a2   = Vectex.Cross(rB,     _axis);

        _axialMass = mA + mB + iA * _a1 * _a1 + iB * _a2 * _a2;
        if (_axialMass > 0.0f) {
          _axialMass = 1.0f / _axialMass;
        }
      }

      // Prismatic constraint.
      {
        _perp = Math.Mul(qA, _localYAxisA);

        _s1 = Vectex.Cross(d + rA, _perp);
        _s2 = Vectex.Cross(rB,     _perp);

        float k11 = mA + mB + iA * _s1 * _s1 + iB * _s2 * _s2;
        float k12 = iA                                  * _s1 + iB * _s2;
        float k22 = iA                                        + iB;
        if (k22 == 0.0f) {
          // For bodies with fixed rotation.
          k22 = 1.0f;
        }
        _K = new Matrix3x2(k11,k12,k12,k22,0,0);
      }

      if (_enableLimit) {
        _translation = Vector2.Dot(_axis, d);
      }
      else {
        _lowerImpulse = 0.0f;
        _upperImpulse = 0.0f;
      }

      if (_enableMotor == false) {
        _motorImpulse = 0.0f;
      }

      if (data.step.warmStarting) {
        // Account for variable time step.
        _impulse      *= data.step.dtRatio;
        _motorImpulse *= data.step.dtRatio;
        _lowerImpulse *= data.step.dtRatio;
        _upperImpulse *= data.step.dtRatio;

        float   axialImpulse = _motorImpulse + _lowerImpulse - _upperImpulse;
        Vector2 P            = _impulse.X * _perp            + axialImpulse * _axis;
        float   LA           = _impulse.X * _s1 + _impulse.Y + axialImpulse * _a1;
        float   LB           = _impulse.X * _s2 + _impulse.Y + axialImpulse * _a2;

        vA -= mA * P;
        wA -= iA * LA;

        vB += mB * P;
        wB += iB * LB;
      }
      else {
        _impulse=Vector2.Zero;
        _motorImpulse = 0.0f;
        _lowerImpulse = 0.0f;
        _upperImpulse = 0.0f;
      }

      data.velocities[_indexA].v = vA;
      data.velocities[_indexA].w = wA;
      data.velocities[_indexB].v = vB;
      data.velocities[_indexB].w = wB;
    }

    internal override void SolveVelocityConstraints(in SolverData data) {
      Vector2 vA = data.velocities[_indexA].v;
      float   wA = data.velocities[_indexA].w;
      Vector2 vB = data.velocities[_indexB].v;
      float   wB = data.velocities[_indexB].w;

      float mA = _invMassA, mB = _invMassB;
      float iA = _invIA,    iB = _invIB;

      // Solve linear motor constraint
      if (_enableMotor) {
        float Cdot       = Vector2.Dot(_axis, vB - vA) + _a2 * wB - _a1 * wA;
        float impulse    = _axialMass * (_motorSpeed - Cdot);
        float oldImpulse = _motorImpulse;
        float maxImpulse = data.step.dt * _maxMotorForce;
        _motorImpulse = System.Math.Clamp(_motorImpulse + impulse, -maxImpulse, maxImpulse);
        impulse       = _motorImpulse - oldImpulse;

        Vector2 P  = impulse * _axis;
        float   LA = impulse * _a1;
        float   LB = impulse * _a2;

        vA -= mA * P;
        wA -= iA * LA;
        vB += mB * P;
        wB += iB * LB;
      }

      if (_enableLimit) {
        // Lower limit
        {
          float C          = _translation                           - _lowerTranslation;
          float Cdot       = Vector2.Dot(_axis, vB - vA) + _a2 * wB - _a1 * wA;
          float impulse    = -_axialMass * (Cdot + MathF.Max(C, 0.0f) * data.step.inv_dt);
          float oldImpulse = _lowerImpulse;
          _lowerImpulse = MathF.Max(_lowerImpulse + impulse, 0.0f);
          impulse       = _lowerImpulse - oldImpulse;

          Vector2 P  = impulse * _axis;
          float   LA = impulse * _a1;
          float   LB = impulse * _a2;

          vA -= mA * P;
          wA -= iA * LA;
          vB += mB * P;
          wB += iB * LB;
        }

        // Upper limit
        // Note: signs are flipped to keep C positive when the constraint is satisfied.
        // This also keeps the impulse positive when the limit is active.
        {
          float C          = _upperTranslation                      - _translation;
          float Cdot       = Vector2.Dot(_axis, vA - vB) + _a1 * wA - _a2 * wB;
          float impulse    = -_axialMass * (Cdot + MathF.Max(C, 0.0f) * data.step.inv_dt);
          float oldImpulse = _upperImpulse;
          _upperImpulse = MathF.Max(_upperImpulse + impulse, 0.0f);
          impulse       = _upperImpulse - oldImpulse;

          Vector2 P  = impulse * _axis;
          float   LA = impulse * _a1;
          float   LB = impulse * _a2;

          vA += mA * P;
          wA += iA * LA;
          vB -= mB * P;
          wB -= iB * LB;
        }
      }

      // Solve the prismatic constraint in block form.
      {
        Vector2 Cdot = new Vector2();
        Cdot.X = Vector2.Dot(_perp, vB - vA) + _s2 * wB - _s1 * wA;
        Cdot.Y = wB                                     - wA;

        Vector2 df = _K.Solve(-Cdot);
        _impulse += df;

        Vector2 P  = df.X * _perp;
        float   LA = df.X * _s1 + df.Y;
        float   LB = df.X * _s2 + df.Y;

        vA -= mA * P;
        wA -= iA * LA;

        vB += mB * P;
        wB += iB * LB;
      }

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

      Rot qA = new Rot(aA), qB = new Rot(aB);

      float mA = _invMassA, mB = _invMassB;
      float iA = _invIA,    iB = _invIB;

      // Compute fresh Jacobians
      Vector2 rA = Math.Mul(qA, _localAnchorA - _localCenterA);
      Vector2 rB = Math.Mul(qB, _localAnchorB - _localCenterB);
      Vector2 d  = cB + rB - cA - rA;

      Vector2 axis = Math.Mul(qA, _localXAxisA);
      float  a1   = Vectex.Cross(d + rA, axis);
      float  a2   = Vectex.Cross(rB,     axis);
      Vector2 perp = Math.Mul(qA, _localYAxisA);

      float s1 = Vectex.Cross(d + rA, perp);
      float s2 = Vectex.Cross(rB,     perp);

      Vector3 impulse;
      Vector2 C1 = new Vector2();
      C1.X = Vector2.Dot(perp, d);
      C1.Y = aB - aA - _referenceAngle;

      float linearError  = MathF.Abs(C1.X);
      float angularError = MathF.Abs(C1.Y);

      bool  active = false;
      float C2     = 0.0f;
      if (_enableLimit) {
        float translation = Vector2.Dot(axis, d);
        if (MathF.Abs(_upperTranslation - _lowerTranslation) < 2.0f * Settings.LinearSlop) {
          C2          = translation;
          linearError = MathF.Max(linearError, MathF.Abs(translation));
          active      = true;
        }
        else if (translation <= _lowerTranslation) {
          C2          = MathF.Min(translation   -_lowerTranslation, 0.0f);
          linearError = MathF.Max(linearError, _lowerTranslation - translation);
          active      = true;
        }
        else if (translation >= _upperTranslation) {
          C2          = MathF.Max(translation - _upperTranslation, 0.0f);
          linearError = MathF.Max(linearError, translation - _upperTranslation);
          active      = true;
        }
      }

      if (active) {
        float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
        float k12 = iA                               * s1 + iB      * s2;
        float k13 = iA * s1                          * a1 + iB * s2 * a2;
        float k22 = iA                                    + iB;
        if (k22 == 0.0f) {
          // For fixed rotation
          k22 = 1.0f;
        }

        float k23 = iA * a1                + iB      * a2;
        float k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

        Mat33 K = new Mat33();
        K.ex = new Vector3(k11, k12, k13);
        K.ey = new Vector3(k12, k22, k23);
        K.ez = new Vector3(k13, k23, k33);

        Vector3 C = new Vector3();
        C.X = C1.X;
        C.Y = C1.Y;
        C.Z = C2;

        impulse = K.Solve33(-C);
      }
      else {
        float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
        float k12 = iA                               * s1 + iB * s2;
        float k22 = iA                                    + iB;
        if (k22 == 0.0f) {
          k22 = 1.0f;
        }

        //Mat22 K = new Mat22();
        Matrix3x2 K = new Matrix3x2(k11, k12, k12, k22, 0,0);
        // K.ex=new Vector2(k11, k12);
        // K.ey=new Vector2(k12, k22);

        Vector2 impulse1 = K.Solve(-C1);
        impulse.X = impulse1.X;
        impulse.Y = impulse1.Y;
        impulse.Z = 0.0f;
      }

      Vector2 P = impulse.X * perp           + impulse.Z * axis;
      float  LA = impulse.X * s1 + impulse.Y + impulse.Z * a1;
      float  LB = impulse.X * s2 + impulse.Y + impulse.Z * a2;

      cA -= mA * P;
      aA -= iA * LA;
      cB += mB * P;
      aB += iB * LB;

      data.positions[_indexA].c = cA;
      data.positions[_indexA].a = aA;
      data.positions[_indexB].c = cB;
      data.positions[_indexB].a = aB;

      return linearError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
    }
  }
}