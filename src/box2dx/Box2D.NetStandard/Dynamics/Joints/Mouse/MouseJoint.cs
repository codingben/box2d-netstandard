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

using System.Numerics;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.World;

namespace Box2D.NetStandard.Dynamics.Joints.Mouse
{
	/// <summary>
	///  A mouse joint is used to make a point on a body track a
	///  specified world point. This a soft constraint with a maximum
	///  force. This allows the constraint to stretch and without
	///  applying huge forces.
	/// </summary>
	public class MouseJoint : Joint
	{
		private readonly float m_dampingRatio;
		private readonly float m_frequencyHz;
		private readonly Vector2 m_localAnchor;
		private readonly float m_maxForce;
		private float m_beta;
		private Vector2 m_C;
		private float m_gamma;
		private Vector2 m_impulse;
		private int m_indexB;
		private float m_invIB;
		private float m_invMassB;
		private Vector2 m_localCenterB;
		private Matrix3x2 m_mass;
		private Vector2 m_rB;
		private Vector2 m_targetA;

		public MouseJoint(MouseJointDef def)
			: base(def)
		{
			m_targetA = def.Target;
			m_localAnchor = Math.MulT(m_bodyB.GetTransform(), m_targetA);

			m_maxForce = def.MaxForce;
			m_impulse = Vector2.Zero;

			m_frequencyHz = def.FrequencyHz;
			m_dampingRatio = def.DampingRatio;

			m_beta = 0.0f;
			m_gamma = 0.0f;
		}

		public override Vector2 GetAnchorA => m_targetA;

		public override Vector2 GetAnchorB => m_bodyB.GetWorldPoint(m_localAnchor);

		public override Vector2 GetReactionForce(float inv_dt) => inv_dt * m_impulse;

		public override float GetReactionTorque(float inv_dt) => inv_dt * 0.0f;

		/// <summary>
		///  Use this to update the target point.
		/// </summary>
		public void SetTarget(Vector2 target)
		{
			if (!m_bodyB.IsAwake())
			{
				m_bodyB.SetAwake(true);
			}

			m_targetA = target;
		}

		internal override void InitVelocityConstraints(in SolverData data)
		{
			m_indexB = m_bodyB.m_islandIndex;
			m_localCenterB = m_bodyB.m_sweep.localCenter;
			m_invMassB = m_bodyB.m_invMass;
			m_invIB = m_bodyB.m_invI;

			Vector2 cB = data.positions[m_indexB].c;
			float aB = data.positions[m_indexB].a;
			Vector2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;

			var qB = new Rot(aB);

			float mass = m_bodyB.GetMass();

			// Frequency
			float omega = Settings.Tau * m_frequencyHz;

			// Damping coefficient
			float d = 2.0f * mass * m_dampingRatio * omega;

			// Spring stiffness
			float k = mass * (omega * omega);

			// magic formulas
			// gamma has units of inverse mass.
			// beta has units of inverse time.
			float h = data.step.dt;
			m_gamma = h * (d + h * k);
			if (m_gamma != 0.0f)
			{
				m_gamma = 1.0f / m_gamma;
			}

			m_beta = h * k * m_gamma;

			// Compute the effective mass matrix.
			m_rB = Math.Mul(qB, -m_localCenterB);

			// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
			//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
			//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
			var K = new Matrix3x2();
			K.M11 = m_invMassB + m_invIB * m_rB.Y * m_rB.Y + m_gamma;
			K.M21 = -m_invIB * m_rB.X * m_rB.Y;
			K.M12 = K.M21;
			K.M22 = m_invMassB + m_invIB * m_rB.X * m_rB.Y + m_gamma;

			/*Matrix3x2*/
			Matrex.Invert(K, out m_mass);

			//_mass = K.GetInverse();

			m_C = cB + m_rB - m_targetA;
			m_C *= m_beta;

			// Cheat with some damping
			wB *= 0.98f;

			if (data.step.warmStarting)
			{
				m_impulse *= data.step.dtRatio;
				vB += m_invMassB * m_impulse;
				wB += m_invIB * Vectex.Cross(m_rB, m_impulse);
			}
			else
			{
				m_impulse = Vector2.Zero;
			}

			data.velocities[m_indexB].v = vB;
			data.velocities[m_indexB].w = wB;
		}

		internal override void SolveVelocityConstraints(in SolverData data)
		{
			Vector2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;

			// Cdot = v + cross(w, r)
			Vector2 Cdot = vB + Vectex.Cross(wB, m_rB);
			var impulse =
				Vector2.Transform(-(Cdot + m_C + m_gamma * m_impulse),
				                  m_mass); //Math.Mul(_mass, -(Cdot + _C + _gamma * _impulse));

			Vector2 oldImpulse = m_impulse;
			m_impulse += impulse;
			float maxImpulse = data.step.dt * m_maxForce;
			if (m_impulse.LengthSquared() > maxImpulse * maxImpulse)
			{
				m_impulse *= maxImpulse / m_impulse.Length();
			}

			impulse = m_impulse - oldImpulse;

			vB += m_invMassB * impulse;
			wB += m_invIB * Vectex.Cross(m_rB, impulse);

			data.velocities[m_indexB].v = vB;
			data.velocities[m_indexB].w = wB;
		}

		internal override bool SolvePositionConstraints(in SolverData data) => true;
	}
}