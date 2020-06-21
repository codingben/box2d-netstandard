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


// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.World;
using Math = Box2D.NetStandard.Common.Math;

namespace Box2D.NetStandard.Dynamics.Joints.Revolute {
  /// <summary>
  /// A revolute joint constrains to bodies to share a common point while they
  /// are free to rotate about the point. The relative rotation about the shared
  /// point is the joint angle. You can limit the relative rotation with
  /// a joint limit that specifies a lower and upper angle. You can use a motor
  /// to drive the relative rotation about the shared point. A maximum motor torque
  /// is provided so that infinite forces are not generated.
  /// </summary>
  public class RevoluteJoint : Joint {
    internal Vector2 _localAnchorA;
    internal Vector2 _localAnchorB;
    private Vector3    _impulse;
    private float   _motorImpulse;

    private bool  _enableMotor;
    private float _maxMotorTorque;
    private float _motorSpeed;

    private bool  _enableLimit;
    internal float _referenceAngle;
    private float _lowerAngle;
    private float _upperAngle;

    private int        _indexA;
    private int        _indexB;
    private Vector2    _rA;
    private Vector2    _rB;
    private Vector2    _localCenterA;
    private Vector2    _localCenterB;
    private float      _invMassA;
    private float      _invMassB;
    private float      _invIA;
    private float      _invIB;
    private Mat33      _mass;      //effective mass for p2p constraint.
    private float      _motorMass; // effective mass for motor/limit angular constraint.
    private LimitState _limitState;

    public override Vector2 GetAnchorA => _bodyA.GetWorldPoint(_localAnchorA);

    public override Vector2 GetAnchorB => _bodyB.GetWorldPoint(_localAnchorB);

    public override Vector2 GetReactionForce(float inv_dt) => inv_dt * new Vector2(_impulse.X, _impulse.Y);

    public override float GetReactionTorque(float inv_dt) => inv_dt * _impulse.Z;

    /// <summary>
    /// Get the current joint angle in radians.
    /// </summary>
    public float JointAngle {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get {
        Body b1 = _bodyA;
        Body b2 = _bodyB;
        return b2._sweep.a - b1._sweep.a - _referenceAngle;
      }
    }


    /// <summary>
    /// Get the current joint angle speed in radians per second.
    /// </summary>
    public float JointSpeed {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get {
        Body b1 = _bodyA;
        Body b2 = _bodyB;
        return b2._angularVelocity - b1._angularVelocity;
      }
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
    /// Get the lower joint limit in radians.
    /// </summary>
    public float LowerLimit {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => _lowerAngle;
    }

    /// <summary>
    /// Get the upper joint limit in radians.
    /// </summary>
    public float UpperLimit {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => _upperAngle;
    }

    /// <summary>
    /// Set the joint limits in radians.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetLimits(float lower, float upper) {
      Debug.Assert(lower <= upper);
      _bodyA.SetAwake(true);
      _bodyB.SetAwake(true);
      _lowerAngle = lower;
      _upperAngle = upper;
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
    /// Get\Set the motor speed in radians per second.
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
    /// Set the maximum motor torque, usually in N-m.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetMaxMotorTorque(float torque) {
      _bodyA.SetAwake(true);
      _bodyB.SetAwake(true);
      _maxMotorTorque = torque;
    }

    /// <summary>
    /// Get the current motor torque, usually in N-m.
    /// </summary>
    public float MotorTorque {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => _motorImpulse;
    }

    public RevoluteJoint(RevoluteJointDef def)
      : base(def) {
      _localAnchorA   = def.LocalAnchorA;
      _localAnchorB   = def.LocalAnchorB;
      _referenceAngle = def.ReferenceAngle;

      _impulse      = new Vector3();
      _motorImpulse = 0.0f;

      _lowerAngle     = def.LowerAngle;
      _upperAngle     = def.UpperAngle;
      _maxMotorTorque = def.MaxMotorTorque;
      _motorSpeed     = def.MotorSpeed;
      _enableLimit    = def.EnableLimit;
      _enableMotor    = def.EnableMotor;
      _limitState      = LimitState.InactiveLimit;
    }

    internal override void InitVelocityConstraints(in SolverData data) {
      _indexA      = _bodyA._islandIndex;
      _indexB      = _bodyB._islandIndex;
      _localCenterA = _bodyA._sweep.localCenter;
      _localCenterB = _bodyB._sweep.localCenter;
      _invMassA     = _bodyA._invMass;
      _invMassB     = _bodyB._invMass;
      _invIA        = _bodyA._invI;
      _invIB        = _bodyB._invI;

      float  aA = data.positions[_indexA].a;
      Vector2 vA = data.velocities[_indexA].v;
      float  wA = data.velocities[_indexA].w;

      float  aB = data.positions[_indexB].a;
      Vector2 vB = data.velocities[_indexB].v;
      float  wB = data.velocities[_indexB].w;

      Rot qA = new Rot(aA), qB = new Rot(aB);

      _rA = Math.Mul(qA, _localAnchorA - _localCenterA);
      _rB = Math.Mul(qB, _localAnchorB - _localCenterB);

      // J = [-I -r1_skew I r2_skew]
      //     [ 0       -1 0       1]
      // r_skew = [-ry; rx]

      // Matlab
      // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
      //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
      //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

      float mA = _invMassA, mB = _invMassB;
      float iA = _invIA,    iB = _invIB;

      bool fixedRotation = (iA + iB == 0.0f);

      _mass.ex.X = mA + mB + _rA.Y * _rA.Y * iA + _rB.Y * _rB.Y * iB;
      _mass.ey.X = -_rA.Y * _rA.X                                 * iA - _rB.Y * _rB.X * iB;
      _mass.ez.X = -_rA.Y                                          * iA - _rB.Y          * iB;
      _mass.ex.Y = _mass.ey.X;
      _mass.ey.Y = mA + mB + _rA.X * _rA.X * iA + _rB.X * _rB.X * iB;
      _mass.ez.Y = _rA.X                                           * iA + _rB.X * iB;
      _mass.ex.Z = _mass.ez.X;
      _mass.ey.Z = _mass.ez.Y;
      _mass.ez.Z = iA + iB;

      _motorMass = iA + iB;
      if (_motorMass > 0.0f) {
        _motorMass = 1.0f / _motorMass;
      }

      if (_enableMotor == false || fixedRotation) {
        _motorImpulse = 0.0f;
      }

      if (_enableLimit && fixedRotation == false) {
        float jointAngle = aB - aA - _referenceAngle;
        if (MathF.Abs(_upperAngle - _lowerAngle) < 2.0f * Settings.AngularSlop) {
          _limitState = LimitState.EqualLimits;
        }
        else if (jointAngle <= _lowerAngle) {
          if (_limitState != LimitState.AtLowerLimit) {
            _impulse.Z = 0.0f;
          }

          _limitState = LimitState.AtLowerLimit;
        }
        else if (jointAngle >= _upperAngle) {
          if (_limitState != LimitState.AtUpperLimit) {
            _impulse.Z = 0.0f;
          }

          _limitState = LimitState.AtUpperLimit;
        }
        else {
          _limitState = LimitState.InactiveLimit;
          _impulse.Z = 0.0f;
        }
      }
      else {
        _limitState = LimitState.InactiveLimit;
      }

      if (data.step.warmStarting) {
        // Scale impulses to support a variable time step.
        _impulse      *= data.step.dtRatio;
        _motorImpulse *= data.step.dtRatio;

        Vector2 P = new Vector2(_impulse.X, _impulse.Y);

        vA -= mA * P;
        wA -= iA * (Vectex.Cross(_rA, P) + _motorImpulse + _impulse.Z);

        vB += mB * P;
        wB += iB * (Vectex.Cross(_rB, P) + _motorImpulse + _impulse.Z);
      }
      else {
        _impulse = Vector3.Zero;
        _motorImpulse = 0.0f;
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

      float mA = _invMassA, mB = _invMassB;
      float iA = _invIA,    iB = _invIB;

      bool fixedRotation = (iA + iB == 0.0f);

      // Solve motor constraint.
      if (_enableMotor && _limitState != LimitState.EqualLimits && fixedRotation == false) {
        float Cdot       = wB - wA - _motorSpeed;
        float impulse    = -_motorMass * Cdot;
        float oldImpulse = _motorImpulse;
        float maxImpulse = data.step.dt * _maxMotorTorque;
        _motorImpulse = System.Math.Clamp(_motorImpulse + impulse, -maxImpulse, maxImpulse);
        impulse        = _motorImpulse - oldImpulse;

        wA -= iA * impulse;
        wB += iB * impulse;
      }

      // Solve limit constraint.
      if (_enableLimit && _limitState != LimitState.InactiveLimit && fixedRotation == false) {
        Vector2 Cdot1 = vB + Vectex.Cross(wB, _rB) - vA - Vectex.Cross(wA, _rA);
        float  Cdot2 = wB                               - wA;
        Vector3   Cdot  = new Vector3(Cdot1.X, Cdot1.Y, Cdot2);

        Vector3 impulse = -_mass.Solve33(Cdot);

        if (_limitState == LimitState.EqualLimits) {
          _impulse += impulse;
        }
        else if (_limitState == LimitState.AtLowerLimit) {
          float newImpulse = _impulse.Z + impulse.Z;
          if (newImpulse < 0.0f) {
            Vector2 rhs     = -Cdot1 + _impulse.Z * new Vector2(_mass.ez.X, _mass.ez.Y);
            Vector2 reduced = _mass.Solve22(rhs);
            impulse.X   =  reduced.X;
            impulse.Y   =  reduced.Y;
            impulse.Z   =  -_impulse.Z;
            _impulse.X += reduced.X;
            _impulse.Y += reduced.Y;
            _impulse.Z =  0.0f;
          }
          else {
            _impulse += impulse;
          }
        }
        else if (_limitState == LimitState.AtUpperLimit) {
          float newImpulse = _impulse.Z + impulse.Z;
          if (newImpulse > 0.0f) {
            Vector2 rhs     = -Cdot1 + _impulse.Z * new Vector2(_mass.ez.X, _mass.ez.Y);
            Vector2 reduced = _mass.Solve22(rhs);
            impulse.X   =  reduced.X;
            impulse.Y   =  reduced.Y;
            impulse.Z   =  -_impulse.Z;
            _impulse.X += reduced.X;
            _impulse.Y += reduced.Y;
            _impulse.Z =  0.0f;
          }
          else {
            _impulse += impulse;
          }
        }

        Vector2 P = new Vector2(impulse.X, impulse.Y);

        vA -= mA * P;
        wA -= iA * (Vectex.Cross(_rA, P) + impulse.Z);

        vB += mB * P;
        wB += iB * (Vectex.Cross(_rB, P) + impulse.Z);
      }
      else {
        // Solve point-to-point constraint
        Vector2 Cdot    = vB + Vectex.Cross(wB, _rB) - vA - Vectex.Cross(wA, _rA);
        Vector2 impulse = _mass.Solve22(-Cdot);

        _impulse.X += impulse.X;
        _impulse.Y += impulse.Y;

        vA -= mA * impulse;
        wA -= iA * Vectex.Cross(_rA, impulse);

        vB += mB * impulse;
        wB += iB * Vectex.Cross(_rB, impulse);
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

      Rot qA = new Rot(aA), qB= new Rot(aB);

      float angularError  = 0.0f;
      float positionError = 0.0f;

      bool fixedRotation = (_invIA + _invIB == 0.0f);

      // Solve angular limit constraint.
      if (_enableLimit && _limitState != LimitState.InactiveLimit && fixedRotation == false) {
        float angle        = aB - aA - _referenceAngle;
        float limitImpulse = 0.0f;

        if (_limitState == LimitState.EqualLimits) {
          // Prevent large angular corrections
          float C = System.Math.Clamp(angle - _lowerAngle, -Settings.MaxAngularCorrection, Settings.MaxAngularCorrection);
          limitImpulse = -_motorMass * C;
          angularError = MathF.Abs(C);
        }
        else if (_limitState == LimitState.AtLowerLimit) {
          float C = angle - _lowerAngle;
          angularError = -C;

          // Prevent large angular corrections and allow some slop.
          C            = System.Math.Clamp(C + Settings.AngularSlop, -Settings.MaxAngularCorrection, 0.0f);
          limitImpulse = -_motorMass * C;
        }
        else if (_limitState ==LimitState.AtUpperLimit) {
          float C = angle - _upperAngle;
          angularError = C;

          // Prevent large angular corrections and allow some slop.
          C            = System.Math.Clamp(C - Settings.AngularSlop, 0.0f, Settings.MaxAngularCorrection);
          limitImpulse = -_motorMass * C;
        }

        aA -= _invIA * limitImpulse;
        aB += _invIB * limitImpulse;
      }

      // Solve point-to-point constraint.
      {
        qA.Set(aA);
        qB.Set(aB);
        Vector2 rA = Math.Mul(qA, _localAnchorA - _localCenterA);
        Vector2 rB = Math.Mul(qB, _localAnchorB - _localCenterB);

        Vector2 C = cB + rB - cA - rA;
        positionError = C.Length();

        float mA = _invMassA, mB = _invMassB;
        float iA = _invIA,    iB = _invIB;

        Matrix3x2 K = new Matrix3x2();
        K.M11 = mA + mB + iA * rA.Y * rA.Y + iB * rB.Y * rB.Y;
        K.M21 = -iA * rA.X * rA.Y - iB * rB.X * rB.Y;
        K.M12 = K.M21;
        K.M22 = mA + mB + iA * rA.X * rA.X + iB * rB.X * rB.X;

        Vector2 impulse = -K.Solve(C);

        cA -= mA * impulse;
        aA -= iA * Vectex.Cross(rA, impulse);

        cB += mB * impulse;
        aB += iB * Vectex.Cross(rB, impulse);
      }

      data.positions[_indexA].c = cA;
      data.positions[_indexA].a = aA;
      data.positions[_indexB].c = cB;
      data.positions[_indexB].a = aB;

      return positionError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
    }
  }
}