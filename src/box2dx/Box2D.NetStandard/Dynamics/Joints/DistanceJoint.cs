/*
  Box2DX Copyright (c) 2008 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com

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

using System.Numerics;
using Box2D.NetStandard.Common;

namespace Box2D.NetStandard.Dynamics.Joints {
  /// <summary>
  /// Distance joint definition. This requires defining an
  /// anchor point on both bodies and the non-zero length of the
  /// distance joint. The definition uses local anchor points
  /// so that the initial configuration can violate the constraint
  /// slightly. This helps when saving and loading a game.
  /// @warning Do not use a zero or short length.
  /// </summary>
  public class DistanceJointDef : JointDef {
    public DistanceJointDef() {
      Type         = JointType.DistanceJoint;
      LocalAnchorA = new Vector2(0.0f, 0.0f);
      LocalAnchorB = new Vector2(0.0f, 0.0f);
      Length       = 1.0f;
      FrequencyHz  = 0.0f;
      DampingRatio = 0.0f;
    }

    /// <summary>
    /// Initialize the bodies, anchors, and length using the world anchors.
    /// </summary>
    public void Initialize(Body.Body body1, Body.Body body2, Vector2 anchor1, Vector2 anchor2) {
      BodyA        = body1;
      BodyB        = body2;
      LocalAnchorA = body1.GetLocalPoint(anchor1);
      LocalAnchorB = body2.GetLocalPoint(anchor2);
      Vector2 d = anchor2 - anchor1;
      Length = d.Length();
    }

    /// <summary>
    /// The local anchor point relative to body1's origin.
    /// </summary>
    public Vector2 LocalAnchorA;

    /// <summary>
    /// The local anchor point relative to body2's origin.
    /// </summary>
    public Vector2 LocalAnchorB;

    /// <summary>
    /// The equilibrium length between the anchor points.
    /// </summary>
    public float Length;

    /// <summary>
    /// The response speed.
    /// </summary>
    public float FrequencyHz;

    /// <summary>
    /// The damping ratio. 0 = no damping, 1 = critical damping.
    /// </summary>
    public float DampingRatio;
  }

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

    public override Vector2 Anchor1 {
      get { return _bodyA.GetWorldPoint(_localAnchorA); }
    }

    public override Vector2 Anchor2 {
      get { return _bodyB.GetWorldPoint(_localAnchorB); }
    }

    public override Vector2 GetReactionForce(float inv_dt) {
      return (inv_dt * _impulse) * _u;
    }

    public override float GetReactionTorque(float inv_dt) {
      return 0.0f;
    }

    public DistanceJoint(DistanceJointDef def)
      : base(def) {
      _localAnchorA = def.LocalAnchorA;
      _localAnchorB = def.LocalAnchorB;
      _length       = def.Length;
      _frequencyHz  = def.FrequencyHz;
      _dampingRatio = def.DampingRatio;
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

      _rA = Common.Math.Mul(qA, _localAnchorA - _localCenterA);
      _rB = Common.Math.Mul(qB, _localAnchorB - _localCenterB);
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
        float omega = 2.0f * Settings.Pi * _frequencyHz;

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

      Body.Body b1 = _bodyA;
      Body.Body b2 = _bodyB;

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