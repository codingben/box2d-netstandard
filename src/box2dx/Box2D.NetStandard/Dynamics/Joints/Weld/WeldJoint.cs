using System;
using System.Numerics;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.World;
using Math = Box2D.NetStandard.Common.Math;
using Vec2 = System.Numerics.Vector2;
using b2Vec2 = System.Numerics.Vector2;
using b2Vec3 = System.Numerics.Vector3;

namespace Box2D.NetStandard.Dynamics.Joints.Weld {
  public class WeldJoint : Joint {
    internal float _frequencyHz;
    internal float _dampingRatio;
    internal float _biios;

    internal Vector2 _localAnchorA;
    internal Vector2 _localAnchorB;
    internal float   _referenceAngle;
    internal float   _gamma;
    internal Vector3 _impulse;

    private int     _indexA;
    private int     _indexB;
    private Vector2 _rA;
    private Vector2 _rB;
    private Vector2 _localCenterA;
    private Vector2 _localCenterB;
    private float   _invMassA;
    private float   _invMassB;
    private float   _invIA;
    private float   _invIB;
    private Mat33   _mass;
    private float   _bias;

    public WeldJoint(WeldJointDef def) : base(def) {
      _localAnchorA   = def.localAnchorA;
      _localAnchorB   = def.localAnchorB;
      _referenceAngle = def.referenceAngle;
      _frequencyHz    = def.frequencyHz;
      _dampingRatio   = def.dampingRatio;
    }

    public override Vector2 GetAnchorA => _bodyA.GetWorldPoint(_localAnchorA);
    public override Vector2 GetAnchorB => _bodyB.GetWorldPoint(_localAnchorB);

    public override Vector2 GetReactionForce(float inv_dt) => inv_dt * new Vector2(_impulse.X, _impulse.Y);

    public override float GetReactionTorque(float inv_dt) => inv_dt * _impulse.Z;


    public void Initialize(Body bA, Body bB, in Vector2 anchor) {
      _bodyA          = bA;
      _bodyB          = bB;
      _localAnchorA   = _bodyA.GetLocalPoint(anchor);
      _localAnchorB   = _bodyB.GetLocalPoint(anchor);
      _referenceAngle = _bodyB.GetAngle() - _bodyA.GetAngle();
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

      float aA = data.positions[_indexA].a;
      Vec2  vA = data.velocities[_indexA].v;
      float wA = data.velocities[_indexA].w;

      float aB = data.positions[_indexB].a;
      Vec2  vB = data.velocities[_indexB].v;
      float wB = data.velocities[_indexB].w;

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

      Mat33 K;
      K.ex.X = mA + mB + _rA.Y * _rA.Y * iA + _rB.Y * _rB.Y * iB;
      K.ey.X = -_rA.Y * _rA.X                               * iA - _rB.Y * _rB.X * iB;
      K.ez.X = -_rA.Y                                       * iA - _rB.Y         * iB;
      K.ex.Y = K.ey.X;
      K.ey.Y = mA + mB + _rA.X * _rA.X * iA + _rB.X * _rB.X * iB;
      K.ez.Y = _rA.X                                        * iA + _rB.X * iB;
      K.ex.Z = K.ez.X;
      K.ey.Z = K.ez.Y;
      K.ez.Z = iA + iB;

      if (_frequencyHz > 0.0f) {
        K.GetInverse22(_mass);

        float invM = iA + iB;
        float m    = invM > 0.0f ? 1.0f / invM : 0.0f;

        float C = aB - aA - _referenceAngle;

        // Frequency
        float omega = 2.0f * MathF.PI * _frequencyHz;

        // Damping coefficient
        float d = 2.0f * m * _dampingRatio * omega;

        // Spring stiffness
        float k = m * omega * omega;

        // magic formulas
        float h = data.step.dt;
        _gamma = h * (d + h * k);
        _gamma = _gamma != 0.0f ? 1.0f / _gamma : 0.0f;
        _bias  = C * h * k * _gamma;

        invM       += _gamma;
        _mass.ez.Z =  invM != 0.0f ? 1.0f / invM : 0.0f;
      }
      else if (K.ez.Z == 0.0f) {
        K.GetInverse22(_mass);
        _gamma = 0.0f;
        _bias  = 0.0f;
      }
      else {
        K.GetSymInverse33(_mass);
        _gamma = 0.0f;
        _bias  = 0.0f;
      }

      if (data.step.warmStarting) {
        // Scale impulses to support a variable time step.
        _impulse *= data.step.dtRatio;

        Vec2 P = new Vector2(_impulse.X, _impulse.Y);

        vA -= mA * P;
        wA -= iA * (Vectex.Cross(_rA, P) + _impulse.Z);

        vB += mB * P;
        wB += iB * (Vectex.Cross(_rB, P) + _impulse.Z);
      }
      else {
        _impulse = Vector3.Zero;
      }

      data.velocities[_indexA].v = vA;
      data.velocities[_indexA].w = wA;
      data.velocities[_indexB].v = vB;
      data.velocities[_indexB].w = wB;
    }


    internal override void SolveVelocityConstraints(in SolverData data) {
      b2Vec2 vA = data.velocities[_indexA].v;
      float  wA = data.velocities[_indexA].w;
      b2Vec2 vB = data.velocities[_indexB].v;
      float  wB = data.velocities[_indexB].w;
      float  mA = _invMassA, mB = _invMassB;

      float iA = _invIA, iB = _invIB;
      if (_frequencyHz > 0.0f) {
        float Cdot2 = wB - wA;

        float impulse2 = -_mass.ez.Z * (Cdot2 + _bias + _gamma * _impulse.Z);
        _impulse.Z += impulse2;

        wA -= iA * impulse2;
        wB += iB * impulse2;

        b2Vec2 Cdot1 = vB + Vectex.Cross(wB, _rB) - vA - Vectex.Cross(wA, _rA);

        b2Vec2 impulse1 = -Math.Mul22(_mass, Cdot1);
        _impulse.X += impulse1.X;
        _impulse.Y += impulse1.Y;

        b2Vec2 P = impulse1;

        vA -= mA * P;
        wA -= iA * Vectex.Cross(_rA, P);

        vB += mB * P;
        wB += iB * Vectex.Cross(_rB, P);
      }

      else {
        b2Vec2 Cdot1 = vB + Vectex.Cross(wB, _rB) - vA - Vectex.Cross(wA, _rA);
        float  Cdot2 = wB                          - wA;
        b2Vec3 Cdot = new Vector3(Cdot1.X, Cdot1.Y, Cdot2);

        b2Vec3 impulse = -Math.Mul(_mass, Cdot);
        _impulse += impulse;

        b2Vec2 P = new Vector2(impulse.X, impulse.Y);

        vA -= mA * P;
        wA -= iA * (Vectex.Cross(_rA, P) + impulse.Z);

        vB += mB * P;
        wB += iB * (Vectex.Cross(_rB, P) + impulse.Z);
      }

      data.velocities[_indexA].v = vA;
      data.velocities[_indexA].w = wA;
      data.velocities[_indexB].v = vB;
      data.velocities[_indexB].w = wB;
    }

    internal override bool SolvePositionConstraints(in SolverData data) {
      b2Vec2 cA = data.positions[_indexA].c;
      float  aA = data.positions[_indexA].a;
      b2Vec2 cB = data.positions[_indexB].c;
      float  aB = data.positions[_indexB].a;

      Rot qA= new Rot(aA), qB = new Rot(aB);

      float mA = _invMassA, mB = _invMassB;
      float iA = _invIA,    iB = _invIB;

      b2Vec2 rA = Math.Mul(qA, _localAnchorA - _localCenterA);
      b2Vec2 rB = Math.Mul(qB, _localAnchorB - _localCenterB);

      float positionError, angularError;

      Mat33 K = new Mat33();
      K.ex.X = mA + mB + rA.Y * rA.Y * iA + rB.Y * rB.Y * iB;
      K.ey.X = -rA.Y * rA.X                             * iA - rB.Y * rB.X * iB;
      K.ez.X = -rA.Y                                    * iA - rB.Y        * iB;
      K.ex.Y = K.ey.X;
      K.ey.Y = mA + mB + rA.X * rA.X * iA + rB.X * rB.X * iB;
      K.ez.Y = rA.X                                     * iA + rB.X * iB;
      K.ex.Z = K.ez.X;
      K.ey.Z = K.ez.Y;
      K.ez.Z = iA + iB;

      if (_frequencyHz > 0.0f)
      {
        b2Vec2 C1 =  cB + rB - cA - rA;

        positionError = C1.Length();
        angularError  = 0.0f;

        b2Vec2 P = -K.Solve22(C1);

        cA -= mA * P;
        aA -= iA * Vectex.Cross(rA, P);

        cB += mB * P;
        aB += iB * Vectex.Cross(rB, P);
      }
      else
      {
        b2Vec2 C1 =  cB + rB - cA - rA;
        float  C2 = aB - aA       - _referenceAngle;

        positionError = C1.Length();
        angularError  = MathF.Abs(C2);

        b2Vec3 C = new Vector3(C1.X, C1.Y, C2);
	
        b2Vec3 impulse;
        if (K.ez.Z > 0.0f)
        {
          impulse = -K.Solve33(C);
        }
        else
        {
          b2Vec2 impulse2 = -K.Solve22(C1);
          impulse = new Vector3(impulse2.X, impulse2.Y, 0.0f);
        }

        b2Vec2 P = new Vector2(impulse.X, impulse.Y);

        cA -= mA * P;
        aA -= iA * (Vectex.Cross(rA, P) + impulse.Z);

        cB += mB * P;
        aB += iB * (Vectex.Cross(rB, P) + impulse.Z);
      }

      data.positions[_indexA].c = cA;
      data.positions[_indexA].a = aA;
      data.positions[_indexB].c = cB;
      data.positions[_indexB].a = aB;

      return positionError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
    }
  }
}