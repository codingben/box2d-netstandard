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

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

using System;
using System.Numerics;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.World;
using Math = Box2D.NetStandard.Common.Math;

namespace Box2D.NetStandard.Dynamics.Joints.Mouse
{
	/// <summary>
	/// A mouse joint is used to make a point on a body track a
	/// specified world point. This a soft constraint with a maximum
	/// force. This allows the constraint to stretch and without
	/// applying huge forces.
	/// </summary>
	public class MouseJoint : Joint
	{
		public float _beta;
		public float _gamma;
		private Vector2 _targetA;
		private Vector2 _localAnchor;
		private float _maxForce;
		private Vector2 _impulse;
		private float _frequencyHz;
		private float _dampingRatio;
		private int _indexB;
		private Vector2 _localCenterB;
		private float _invMassB;
		private float _invIB;
		private Vector2 _rB;
		private Vector2 _localAnchorB;
		// private Mat22 _mass;
		private Matrix3x2 _mass;
		private Vector2 _C;

		public override Vector2 GetAnchorA => _targetA;

		public override Vector2 GetAnchorB => _bodyB.GetWorldPoint(_localAnchor);

		public override Vector2 GetReactionForce(float inv_dt)
		{
			return inv_dt * _impulse;
		}

		public override float GetReactionTorque(float inv_dt)
		{
			return inv_dt * 0.0f;
		}

		/// <summary>
		/// Use this to update the target point.
		/// </summary>
		public void SetTarget(Vector2 target)
		{
			if (!_bodyB.IsAwake())
			{
				_bodyB.SetAwake(true);
			}
			_targetA = target;
		}

		public MouseJoint(MouseJointDef def)
			: base(def)
		{
			_targetA = def.Target;
			_localAnchor = Math.MulT(_bodyB.GetTransform(), _targetA);

			_maxForce = def.MaxForce;
			_impulse = Vector2.Zero;

			_frequencyHz = def.FrequencyHz;
			_dampingRatio = def.DampingRatio;

			_beta = 0.0f;
			_gamma = 0.0f;
		}

		internal override void InitVelocityConstraints(in SolverData data)
		{
			_indexB       = _bodyB._islandIndex;
			_localCenterB = _bodyB._sweep.localCenter;
			_invMassB     = _bodyB._invMass;
			_invIB        = _bodyB._invI;

			Vector2 cB =  data.positions[_indexB].c;
			float  aB =  data.positions[_indexB].a;
			Vector2 vB = data.velocities[_indexB].v;
			float  wB = data.velocities[_indexB].w;

			Rot qB = new Rot(aB);

			float mass = _bodyB.GetMass();

			// Frequency
			float omega = Settings.Tau * _frequencyHz;

			// Damping coefficient
			float d = 2.0f * mass * _dampingRatio * omega;

			// Spring stiffness
			float k = mass * (omega * omega);

			// magic formulas
			// gamma has units of inverse mass.
			// beta has units of inverse time.
			float h = data.step.dt;
			_gamma = h * (d + h * k);
			if (_gamma != 0.0f)
			{
				_gamma = 1.0f / _gamma;
			}
			_beta = h * k * _gamma;

			// Compute the effective mass matrix.
			_rB = Math.Mul(qB, _localAnchorB - _localCenterB);

			// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
			//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
			//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
			Matrix3x2 K = new Matrix3x2();
			K.M11 = _invMassB + _invIB * _rB.Y * _rB.Y + _gamma;
			K.M21 = -_invIB * _rB.X * _rB.Y;
			K.M12 = K.M21;
			K.M22 = _invMassB + _invIB * _rB.X * _rB.Y + _gamma;

			/*Matrix3x2*/ Matrex.Invert(K, out _mass);
			
			//_mass = K.GetInverse();

			_C =  cB + _rB - _targetA;
			_C *= _beta;

			// Cheat with some damping
			wB *= 0.98f;

			if (data.step.warmStarting)
			{
				_impulse *= data.step.dtRatio;
				vB        += _invMassB * _impulse;
				wB        += _invIB    * Vectex.Cross(_rB, _impulse);
			}
			else
			{
				_impulse=Vector2.Zero;
			}

			data.velocities[_indexB].v = vB;
			data.velocities[_indexB].w = wB;
		}

		internal override void SolveVelocityConstraints(in SolverData data)
		{
			Vector2 vB = data.velocities[_indexB].v;
			float  wB = data.velocities[_indexB].w;

			// Cdot = v + cross(w, r)
			Vector2 Cdot    = vB + Vectex.Cross(wB, _rB);
			Vector2 impulse = Vector2.Transform(-(Cdot + _C + (float)_gamma * _impulse),_mass); //Math.Mul(_mass, -(Cdot + _C + _gamma * _impulse));

			Vector2 oldImpulse = _impulse;
			_impulse += impulse;
			float maxImpulse = data.step.dt * _maxForce;
			if (_impulse.LengthSquared() > maxImpulse * maxImpulse)
			{
				_impulse *= maxImpulse / _impulse.Length();
			}
			impulse = _impulse - oldImpulse;

			vB += _invMassB * impulse;
			wB += _invIB    * Vectex.Cross(_rB, impulse);

			data.velocities[_indexB].v = vB;
			data.velocities[_indexB].w = wB;
		}

		internal override bool SolvePositionConstraints(in SolverData data) => true;
	}
}
