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
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.World;
using Math = Box2D.NetStandard.Common.Math;

namespace Box2D.NetStandard.Dynamics.Joints.Pulley
{
    /// <summary>
    ///  The pulley joint is connected to two bodies and two fixed ground points.
    ///  The pulley supports a ratio such that:
    ///  length1 + ratio * length2 <= constant
    ///  Yes, the force transmitted is scaled by the ratio.
    ///  The pulley also enforces a maximum length limit on both sides. This is
    ///  useful to prevent one side of the pulley hitting the top.
    /// </summary>
    public class PulleyJoint : Joint
    {
        public static readonly float MinPulleyLength = 2.0f;
        private readonly float m_constant;

        private readonly Vector2 m_localAnchorA;
        private readonly Vector2 m_localAnchorB;
        private float m_impulse;

        private int m_indexA;
        private int m_indexB;
        private float m_invIA;
        private float m_invIB;
        private float m_invMassA;
        private float m_invMassB;
        private Vector2 m_localCenterA;
        private Vector2 m_localCenterB;
        private float m_mass;
        private Vector2 m_rA;
        private Vector2 m_rB;
        private Vector2 m_uA;
        private Vector2 m_uB;

        public PulleyJoint(PulleyJointDef def)
            : base(def)
        {
            GroundAnchorA = def.GroundAnchorA;
            GroundAnchorB = def.GroundAnchorB;
            m_localAnchorA = def.LocalAnchorA;
            m_localAnchorB = def.LocalAnchorB;

            LengthA = def.LengthA;
            LengthB = def.LengthB;

            //Debug.Assert(def.Ratio != 0.0f);
            Ratio = def.Ratio;

            m_constant = def.LengthA + Ratio * def.LengthB;

            m_impulse = 0.0f;
        }

        public override Vector2 GetAnchorA => m_bodyA.GetWorldPoint(m_localAnchorA);

        public override Vector2 GetAnchorB => m_bodyB.GetWorldPoint(m_localAnchorB);

        /// <summary>
        ///  Get the first ground anchor.
        /// </summary>
        public Vector2 GroundAnchorA
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
        }

        /// <summary>
        ///  Get the second ground anchor.
        /// </summary>
        public Vector2 GroundAnchorB
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
        }

        /// <summary>
        ///  Get the current length of the segment attached to body1.
        /// </summary>
        public float LengthA
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
        }

        /// <summary>
        ///  Get the current length of the segment attached to body2.
        /// </summary>
        public float LengthB
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
        }

        /// <summary>
        ///  Get the pulley ratio.
        /// </summary>
        public float Ratio
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
        }

        public override Vector2 GetReactionForce(float invDt) => invDt * m_impulse * m_uB;

        public override float GetReactionTorque(float inv_dt) => 0.0f;

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

            Vector2 cA = data.positions[m_indexA].c;
            float aA = data.positions[m_indexA].a;
            Vector2 vA = data.velocities[m_indexA].v;
            float wA = data.velocities[m_indexA].w;

            Vector2 cB = data.positions[m_indexB].c;
            float aB = data.positions[m_indexB].a;
            Vector2 vB = data.velocities[m_indexB].v;
            float wB = data.velocities[m_indexB].w;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            m_rA = Math.Mul(qA, m_localAnchorA - m_localCenterA);
            m_rB = Math.Mul(qB, m_localAnchorB - m_localCenterB);

            // Get the pulley axes.
            m_uA = cA + m_rA - GroundAnchorA;
            m_uB = cB + m_rB - GroundAnchorB;

            float lengthA = m_uA.Length();
            float lengthB = m_uB.Length();

            if (lengthA > 10.0f * Settings.LinearSlop)
            {
                m_uA *= 1.0f / lengthA;
            }
            else
            {
                m_uA = Vector2.Zero;
            }

            if (lengthB > 10.0f * Settings.LinearSlop)
            {
                m_uB *= 1.0f / lengthB;
            }
            else
            {
                m_uB = Vector2.Zero;
            }

            // Compute effective mass.
            float ruA = Vectex.Cross(m_rA, m_uA);
            float ruB = Vectex.Cross(m_rB, m_uB);

            float mA = m_invMassA + m_invIA * ruA * ruA;
            float mB = m_invMassB + m_invIB * ruB * ruB;

            m_mass = mA + Ratio * Ratio * mB;

            if (m_mass > 0.0f)
            {
                m_mass = 1.0f / m_mass;
            }

            if (data.step.warmStarting)
            {
                // Scale impulses to support variable time steps.
                m_impulse *= data.step.dtRatio;

                // Warm starting.
                Vector2 PA = -m_impulse * m_uA;
                Vector2 PB = -Ratio * m_impulse * m_uB;

                vA += m_invMassA * PA;
                wA += m_invIA * Vectex.Cross(m_rA, PA);
                vB += m_invMassB * PB;
                wB += m_invIB * Vectex.Cross(m_rB, PB);
            }
            else
            {
                m_impulse = 0.0f;
            }

            data.velocities[m_indexA].v = vA;
            data.velocities[m_indexA].w = wA;
            data.velocities[m_indexB].v = vB;
            data.velocities[m_indexB].w = wB;
        }

        internal override void SolveVelocityConstraints(in SolverData data)
        {
            Vector2 vA = data.velocities[m_indexA].v;
            float wA = data.velocities[m_indexA].w;
            Vector2 vB = data.velocities[m_indexB].v;
            float wB = data.velocities[m_indexB].w;

            Vector2 vpA = vA + Vectex.Cross(wA, m_rA);
            Vector2 vpB = vB + Vectex.Cross(wB, m_rB);

            float Cdot = -Vector2.Dot(m_uA, vpA) - Ratio * Vector2.Dot(m_uB, vpB);
            float impulse = -m_mass * Cdot;
            m_impulse += impulse;

            Vector2 PA = -impulse * m_uA;
            Vector2 PB = -Ratio * impulse * m_uB;
            vA += m_invMassA * PA;
            wA += m_invIA * Vectex.Cross(m_rA, PA);
            vB += m_invMassB * PB;
            wB += m_invIB * Vectex.Cross(m_rB, PB);

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

            Rot qA = new Rot(aA), qB = new Rot(aB);

            Vector2 rA = Math.Mul(qA, m_localAnchorA - m_localCenterA);
            Vector2 rB = Math.Mul(qB, m_localAnchorB - m_localCenterB);

            // Get the pulley axes.
            Vector2 uA = cA + rA - GroundAnchorA;
            Vector2 uB = cB + rB - GroundAnchorB;

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

            float mA = m_invMassA + m_invIA * ruA * ruA;
            float mB = m_invMassB + m_invIB * ruB * ruB;

            float mass = mA + Ratio * Ratio * mB;

            if (mass > 0.0f)
            {
                mass = 1.0f / mass;
            }

            float C = m_constant - lengthA - Ratio * lengthB;
            float linearError = MathF.Abs(C);

            float impulse = -mass * C;

            Vector2 PA = -impulse * uA;
            Vector2 PB = -Ratio * impulse * uB;

            cA += m_invMassA * PA;
            aA += m_invIA * Vectex.Cross(rA, PA);
            cB += m_invMassB * PB;
            aB += m_invIB * Vectex.Cross(rB, PB);

            data.positions[m_indexA].c = cA;
            data.positions[m_indexA].a = aA;
            data.positions[m_indexB].c = cB;
            data.positions[m_indexB].a = aB;

            return linearError < Settings.LinearSlop;
        }
    }
}