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
// d = pB - pA = xB + rB - xA - rA
// C = dot(ay, d)
// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

// Spring linear constraint
// C = dot(ax, d)
// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

// Motor rotational constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]

using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.World;
using Math = Box2D.NetStandard.Common.Math;

namespace Box2D.NetStandard.Dynamics.Joints.Wheel
{
	public class WheelJoint : Joint, IMotorisedJoint
	{
		private readonly Vector2 m_localAnchorA;
		private readonly Vector2 m_localAnchorB;
		private readonly Vector2 m_localXAxisA;
		private readonly Vector2 m_localYAxisA;

		private Vector2 m_ax, m_ay;
		private float m_axialMass;

		private float m_bias;
		private float m_damping;

		private bool m_enableLimit;
		private bool m_enableMotor;
		private float m_gamma;
		private float m_impulse;

		private int m_indexA;
		private int m_indexB;
		private float m_invIA;
		private float m_invIB;
		private float m_invMassA;
		private float m_invMassB;
		private Vector2 m_localCenterA;
		private Vector2 m_localCenterB;

		private float m_lowerImpulse;
		private float m_lowerTranslation;

		private float m_mass;

		private float m_maxMotorTorque;

		private float m_motorImpulse;
		private float m_motorMass;
		private float m_motorSpeed;
		private float m_sAx, m_sBx;
		private float m_sAy, m_sBy;
		private float m_springImpulse;
		private float m_springMass;

		private float m_stiffness;
		private float m_translation;
		private float m_upperImpulse;
		private float m_upperTranslation;

		public WheelJoint(WheelJointDef def) : base(def)
		{
			m_localAnchorA = def.localAnchorA;
			m_localAnchorB = def.localAnchorB;
			m_localXAxisA = def.localAxisA;
			m_localYAxisA = Vectex.Cross(1f, m_localXAxisA);

			m_mass = 0f;
			m_impulse = 0f;
			m_motorMass = 0f;
			m_motorImpulse = 0f;
			m_springMass = 0f;
			m_springImpulse = 0f;

			m_axialMass = 0f;
			m_lowerImpulse = 0f;
			m_upperImpulse = 0f;
			m_lowerTranslation = def.lowerTranslation;
			m_upperTranslation = def.upperTranslation;
			m_enableLimit = def.enableLimit;

			m_maxMotorTorque = def.maxMotorTorque;
			m_motorSpeed = def.motorSpeed;
			m_enableMotor = def.enableMotor;

			m_bias = 0f;
			m_gamma = 0f;

			m_ax = Vector2.Zero;
			m_ay = Vector2.Zero;

			if (def.frequencyHz.HasValue && def.dampingRatio.HasValue)
			{
				LinearStiffness(out def.stiffness, out def.damping, def.frequencyHz.Value, def.dampingRatio.Value, def.bodyA,
				                def.bodyB);
			}

			m_stiffness = def.stiffness;
			m_damping = def.damping;
		}

		public override Vector2 GetAnchorA => m_bodyA.GetWorldPoint(m_localAnchorA);

		public override Vector2 GetAnchorB => m_bodyB.GetWorldPoint(m_localAnchorB);

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void SetMotorSpeed(float speed)
		{
			if (speed != m_motorSpeed)
			{
				m_bodyA.SetAwake(true);
				m_bodyB.SetAwake(true);
				m_motorSpeed = speed;
			}
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public float GetMotorSpeed() => m_motorSpeed;

		public float MotorSpeed
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => GetMotorSpeed();
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			set => SetMotorSpeed(value);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public override Vector2 GetReactionForce(float inv_dt) => inv_dt * (m_impulse * m_ay + m_springImpulse * m_ax);

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public override float GetReactionTorque(float inv_dt) => inv_dt * m_motorImpulse;

		public float GetJointTranslation()
		{
			Body bA = m_bodyA;
			Body bB = m_bodyB;

			Vector2 pA = bA.GetWorldPoint(m_localAnchorA);
			Vector2 pB = bB.GetWorldPoint(m_localAnchorB);
			Vector2 d = pB - pA;
			Vector2 axis = bA.GetWorldVector(m_localXAxisA);

			return Vector2.Dot(d, axis);
		}

		public float GetJointLinearSpeed()
		{
			Body bA = m_bodyA;
			Body bB = m_bodyB;

			var rA =
				Vector2.Transform(m_localAnchorA - bA.m_sweep.localCenter,
				                  bA.m_xf.q); // Math.Mul(bA._xf.q, _localAnchorA - bA._sweep.localCenter);
			var rB =
				Vector2.Transform(m_localAnchorB - bB.m_sweep.localCenter,
				                  bB.m_xf.q); // Math.Mul(bB._xf.q, _localAnchorB - bB._sweep.localCenter);
			Vector2 p1 = bA.m_sweep.c + rA;
			Vector2 p2 = bB.m_sweep.c + rB;
			Vector2 d = p2 - p1;
			var axis = Vector2.Transform(m_localXAxisA, bA.m_xf.q); //Math.Mul(bA._xf.q, _localXAxisA);

			Vector2 vA = bA.m_linearVelocity;
			Vector2 vB = bB.m_linearVelocity;
			float wA = bA.m_angularVelocity;
			float wB = bB.m_angularVelocity;

			return Vector2.Dot(d, Vectex.Cross(wA, axis)) +
			       Vector2.Dot(axis, vB + Vectex.Cross(wB, rB) - vA - Vectex.Cross(wA, rA));
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public float GetJointAngle() => m_bodyB.m_sweep.a - m_bodyA.m_sweep.a;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public float GetJointAngularSpeed() => m_bodyB.m_angularVelocity - m_bodyA.m_angularVelocity;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public bool IsLimitEnabled() => m_enableLimit;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void EnableLimit(bool flag)
		{
			if (flag != m_enableLimit)
			{
				m_bodyA.SetAwake(true);
				m_bodyB.SetAwake(true);
				m_enableLimit = flag;
				m_lowerImpulse = 0.0f;
				m_upperImpulse = 0.0f;
			}
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private float GetLowerLimit() => m_lowerTranslation;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private float GetUpperLimit() => m_upperTranslation;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private void SetLimits(float lower, float upper)
		{
			//Debug.Assert(lower <= upper);
			if (lower != m_lowerTranslation || upper != m_upperTranslation)
			{
				m_bodyA.SetAwake(true);
				m_bodyB.SetAwake(true);
				m_lowerTranslation = lower;
				m_upperTranslation = upper;
				m_lowerImpulse = 0.0f;
				m_upperImpulse = 0.0f;
			}
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private bool IsMotorEnabled() => m_enableMotor;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private void EnableMotor(bool flag)
		{
			if (flag != m_enableMotor)
			{
				m_bodyA.SetAwake(true);
				m_bodyB.SetAwake(true);
				m_enableMotor = flag;
			}
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private void SetMaxMotorTorque(float torque)
		{
			if (torque != m_maxMotorTorque)
			{
				m_bodyA.SetAwake(true);
				m_bodyB.SetAwake(true);
				m_maxMotorTorque = torque;
			}
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private float GetMotorTorque(float inv_dt) => inv_dt * m_motorImpulse;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private void SetStiffness(float stiffness)
		{
			m_stiffness = stiffness;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private float GetStiffness() => m_stiffness;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private void SetDamping(float damping)
		{
			m_damping = damping;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		private float GetDamping() => m_damping;

		internal override void InitVelocityConstraints(in SolverData data)
		{
			m_indexA = m_bodyA.m_islandIndex;
			m_indexB = m_bodyB.m_islandIndex;
			m_localCenterA = m_bodyA.m_sweep.localCenter;
			m_localCenterB = m_bodyB.m_sweep.localCenter;
			m_invMassA = m_bodyA.m_invMass;
			m_invMassB = m_bodyB.m_invMass;
			m_invIA = m_bodyA.m_invI;
			m_invIB = m_bodyB.m_invI;

			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;

			Vector2 cA = data.positions[m_indexA].c;
			float aA = data.positions[m_indexA].a;
			Vector2 vA = data.velocities[m_indexA].v;
			float wA = data.velocities[m_indexA].w;

			Vector2 cB = data.positions[m_indexB].c;
			float aB = data.positions[m_indexB].a;
			Vector2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;

			Rot qA = new Rot(aA), qB = new Rot(aB);

			// Compute the effective masses.
			Vector2 rA = Math.Mul(qA, m_localAnchorA - m_localCenterA);
			Vector2 rB = Math.Mul(qB, m_localAnchorB - m_localCenterB);
			Vector2 d = cB + rB - cA - rA;

			// Point to line constraint
			{
				m_ay = Math.Mul(qA, m_localYAxisA);
				m_sAy = Vectex.Cross(d + rA, m_ay);
				m_sBy = Vectex.Cross(rB, m_ay);

				m_mass = mA + mB + iA * m_sAy * m_sAy + iB * m_sBy * m_sBy;

				if (m_mass > 0.0f)
				{
					m_mass = 1.0f / m_mass;
				}
			}

			// Spring constraint
			m_ax = Math.Mul(qA, m_localXAxisA);
			m_sAx = Vectex.Cross(d + rA, m_ax);
			m_sBx = Vectex.Cross(rB, m_ax);

			float invMass = mA + mB + iA * m_sAx * m_sAx + iB * m_sBx * m_sBx;
			if (invMass > 0.0f)
			{
				m_axialMass = 1.0f / invMass;
			}
			else
			{
				m_axialMass = 0.0f;
			}

			m_springMass = 0.0f;
			m_bias = 0.0f;
			m_gamma = 0.0f;

			if (m_stiffness > 0.0f && invMass > 0.0f)
			{
				m_springMass = 1.0f / invMass;

				float C = Vector2.Dot(d, m_ax);

				// magic formulas
				float h = data.step.dt;
				m_gamma = h * (m_damping + h * m_stiffness);
				if (m_gamma > 0.0f)
				{
					m_gamma = 1.0f / m_gamma;
				}

				m_bias = C * h * m_stiffness * m_gamma;

				m_springMass = invMass + m_gamma;
				if (m_springMass > 0.0f)
				{
					m_springMass = 1.0f / m_springMass;
				}
			}
			else
			{
				m_springImpulse = 0.0f;
			}

			if (m_enableLimit)
			{
				m_translation = Vector2.Dot(m_ax, d);
			}
			else
			{
				m_lowerImpulse = 0.0f;
				m_upperImpulse = 0.0f;
			}

			if (m_enableMotor)
			{
				m_motorMass = iA + iB;
				if (m_motorMass > 0.0f)
				{
					m_motorMass = 1.0f / m_motorMass;
				}
			}
			else
			{
				m_motorMass = 0.0f;
				m_motorImpulse = 0.0f;
			}

			if (data.step.warmStarting)
			{
				// Account for variable time step.
				m_impulse *= data.step.dtRatio;
				m_springImpulse *= data.step.dtRatio;
				m_motorImpulse *= data.step.dtRatio;

				float axialImpulse = m_springImpulse + m_lowerImpulse - m_upperImpulse;
				Vector2 P = m_impulse * m_ay + axialImpulse * m_ax;
				float LA = m_impulse * m_sAy + axialImpulse * m_sAx + m_motorImpulse;
				float LB = m_impulse * m_sBy + axialImpulse * m_sBx + m_motorImpulse;

				vA -= m_invMassA * P;
				wA -= m_invIA * LA;

				vB += m_invMassB * P;
				wB += m_invIB * LB;
			}
			else
			{
				m_impulse = 0.0f;
				m_springImpulse = 0.0f;
				m_motorImpulse = 0.0f;
				m_lowerImpulse = 0.0f;
				m_upperImpulse = 0.0f;
			}

			data.velocities[m_indexA].v = vA;
			data.velocities[m_indexA].w = wA;
			data.velocities[m_indexB].v = vB;
			data.velocities[m_indexB].w = wB;
		}

		internal override void SolveVelocityConstraints(in SolverData data)
		{
			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;

			Vector2 vA = data.velocities[m_indexA].v;
			float wA = data.velocities[m_indexA].w;
			Vector2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;

			// Solve spring constraint
			{
				float Cdot = Vector2.Dot(m_ax, vB - vA) + m_sBx * wB - m_sAx * wA;
				float impulse = -m_springMass * (Cdot + m_bias + m_gamma * m_springImpulse);
				m_springImpulse += impulse;

				Vector2 P = impulse * m_ax;
				float LA = impulse * m_sAx;
				float LB = impulse * m_sBx;

				vA -= mA * P;
				wA -= iA * LA;

				vB += mB * P;
				wB += iB * LB;
			}

			// Solve rotational motor constraint
			{
				float Cdot = wB - wA - m_motorSpeed;
				float impulse = -m_motorMass * Cdot;

				float oldImpulse = m_motorImpulse;
				float maxImpulse = data.step.dt * m_maxMotorTorque;
				m_motorImpulse = System.Math.Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
				impulse = m_motorImpulse - oldImpulse;

				wA -= iA * impulse;
				wB += iB * impulse;
			}

			if (m_enableLimit)
			{
				// Lower limit
				{
					float C = m_translation - m_lowerTranslation;
					float Cdot = Vector2.Dot(m_ax, vB - vA) + m_sBx * wB - m_sAx * wA;
					float impulse = -m_axialMass * (Cdot + MathF.Max(C, 0.0f) * data.step.inv_dt);
					float oldImpulse = m_lowerImpulse;
					m_lowerImpulse = MathF.Max(m_lowerImpulse + impulse, 0.0f);
					impulse = m_lowerImpulse - oldImpulse;

					Vector2 P = impulse * m_ax;
					float LA = impulse * m_sAx;
					float LB = impulse * m_sBx;

					vA -= mA * P;
					wA -= iA * LA;
					vB += mB * P;
					wB += iB * LB;
				}

				// Upper limit
				// Note: signs are flipped to keep C positive when the constraint is satisfied.
				// This also keeps the impulse positive when the limit is active.
				{
					float C = m_upperTranslation - m_translation;
					float Cdot = Vector2.Dot(m_ax, vA - vB) + m_sAx * wA - m_sBx * wB;
					float impulse = -m_axialMass * (Cdot + MathF.Max(C, 0.0f) * data.step.inv_dt);
					float oldImpulse = m_upperImpulse;
					m_upperImpulse = MathF.Max(m_upperImpulse + impulse, 0.0f);
					impulse = m_upperImpulse - oldImpulse;

					Vector2 P = impulse * m_ax;
					float LA = impulse * m_sAx;
					float LB = impulse * m_sBx;

					vA += mA * P;
					wA += iA * LA;
					vB -= mB * P;
					wB -= iB * LB;
				}
			}

			// Solve point to line constraint
			{
				float Cdot = Vector2.Dot(m_ay, vB - vA) + m_sBy * wB - m_sAy * wA;
				float impulse = -m_mass * Cdot;
				m_impulse += impulse;

				Vector2 P = impulse * m_ay;
				float LA = impulse * m_sAy;
				float LB = impulse * m_sBy;

				vA -= mA * P;
				wA -= iA * LA;

				vB += mB * P;
				wB += iB * LB;
			}

			data.velocities[m_indexA].v = vA;
			data.velocities[m_indexA].w = wA;
			data.velocities[m_indexB].v = vB;
			data.velocities[m_indexB].w = wB;
		}

		internal override bool SolvePositionConstraints(in SolverData data)
		{
			Vector2 cA = data.positions[m_indexA].c;
			float aA = data.positions[m_indexA].a;
			Vector2 cB = data.positions[m_indexB].c;
			float aB = data.positions[m_indexB].a;

			var linearError = 0.0f;

			if (m_enableLimit)
			{
				Rot qA = new Rot(aA), qB = new Rot(aB);

				Vector2 rA = Math.Mul(qA, m_localAnchorA - m_localCenterA);
				Vector2 rB = Math.Mul(qB, m_localAnchorB - m_localCenterB);
				Vector2 d = cB - cA + rB - rA;

				Vector2 ax = Math.Mul(qA, m_localXAxisA);
				float sAx = Vectex.Cross(d + rA, m_ax);
				float sBx = Vectex.Cross(rB, m_ax);

				var C = 0.0f;
				float translation = Vector2.Dot(ax, d);
				if (MathF.Abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings.LinearSlop)
				{
					C = translation;
				}
				else if (translation <= m_lowerTranslation)
				{
					C = MathF.Min(translation - m_lowerTranslation, 0.0f);
				}
				else if (translation >= m_upperTranslation)
				{
					C = MathF.Max(translation - m_upperTranslation, 0.0f);
				}

				if (C != 0.0f)
				{
					float invMass = m_invMassA + m_invMassB + m_invIA * sAx * sAx + m_invIB * sBx * sBx;
					var impulse = 0.0f;
					if (invMass != 0.0f)
					{
						impulse = -C / invMass;
					}

					Vector2 P = impulse * ax;
					float LA = impulse * sAx;
					float LB = impulse * sBx;

					cA -= m_invMassA * P;
					aA -= m_invIA * LA;
					cB += m_invMassB * P;
					aB += m_invIB * LB;

					linearError = MathF.Abs(C);
				}
			}

			// Solve perpendicular constraint
			{
				Rot qA = new Rot(aA), qB = new Rot(aB);

				Vector2 rA = Math.Mul(qA, m_localAnchorA - m_localCenterA);
				Vector2 rB = Math.Mul(qB, m_localAnchorB - m_localCenterB);
				Vector2 d = cB - cA + rB - rA;

				Vector2 ay = Math.Mul(qA, m_localYAxisA);

				float sAy = Vectex.Cross(d + rA, ay);
				float sBy = Vectex.Cross(rB, ay);

				float C = Vector2.Dot(d, ay);

				float invMass = m_invMassA + m_invMassB + m_invIA * m_sAy * m_sAy + m_invIB * m_sBy * m_sBy;

				var impulse = 0.0f;
				if (invMass != 0.0f)
				{
					impulse = -C / invMass;
				}

				Vector2 P = impulse * ay;
				float LA = impulse * sAy;
				float LB = impulse * sBy;

				cA -= m_invMassA * P;
				aA -= m_invIA * LA;
				cB += m_invMassB * P;
				aB += m_invIB * LB;

				linearError = MathF.Max(linearError, MathF.Abs(C));
			}

			data.positions[m_indexA].c = cA;
			data.positions[m_indexA].a = aA;
			data.positions[m_indexB].c = cB;
			data.positions[m_indexB].a = aB;

			return linearError <= Settings.LinearSlop;
		}
	}
}