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
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.World;

namespace Box2D.NetStandard.Dynamics.Joints.Distance
{
    /// <summary>
    ///  A distance joint constrains two points on two bodies
    ///  to remain at a fixed distance from each other. You can view
    ///  this as a massless, rigid rod.
    /// </summary>
    public class DistanceJoint : Joint
    {
        private readonly float m_length;
        private readonly Vector2 m_localAnchorA;
        private readonly Vector2 m_localAnchorB;
        private float m_bias;
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
        private float m_mass; // effective mass for the constraint.
        private Vector2 m_rA;
        private Vector2 m_rB;
        private Vector2 m_u;

        public DistanceJoint(DistanceJointDef def)
            : base(def)
        {
            m_localAnchorA = def.localAnchorA;
            m_localAnchorB = def.localAnchorB;
            m_length = def.length;

            if (def.frequencyHz.HasValue && def.dampingRatio.HasValue)
            {
                LinearStiffness(out def.stiffness, out def.damping, def.frequencyHz.Value, def.dampingRatio.Value, def.bodyA,
                                def.bodyB);
            }

            Stiffness = def.stiffness;
            Damping = def.damping;
            m_impulse = 0.0f;
            m_gamma = 0.0f;
            m_bias = 0.0f;
        }

        public override Vector2 GetAnchorA => m_bodyA.GetWorldPoint(m_localAnchorA);

        public override Vector2 GetAnchorB => m_bodyB.GetWorldPoint(m_localAnchorB);

        /// <summary>
        ///  Set/get the linear stiffness in N/m
        /// </summary>
        public float Stiffness
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set;
        }

        /// <summary>
        ///  Set/get linear damping in N*s/m
        /// </summary>
        public float Damping
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override Vector2 GetReactionForce(float inv_dt) => inv_dt * m_impulse * m_u;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
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
            m_u = cB + m_rB - cA - m_rA;

            // Handle singularity.
            float length = m_u.Length();
            if (length > Settings.LinearSlop)
            {
                m_u *= 1.0f / length;
            }
            else
            {
                m_u = Vector2.Zero;
            }

            float crAu = Vectex.Cross(m_rA, m_u);
            float crBu = Vectex.Cross(m_rB, m_u);
            float invMass = m_invMassA + m_invIA * crAu * crAu + m_invMassB + m_invIB * crBu * crBu;

            // Compute the effective mass matrix.
            if (Stiffness > 0.0f)
            {
                float C = length - m_length;

                float d = Damping;

                float k = Stiffness;

                // magic formulas
                float h = data.step.dt;

                // gamma = 1 / (h * (d + h * k)), the extra factor of h in the denominator is since the lambda is an impulse, not a force
                m_gamma = h * (d + h * k);
                m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
                m_bias = C * h * k * m_gamma;

                invMass += m_gamma;
                m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;
            }
            else
            {
                m_gamma = 0.0f;
                m_bias = 0.0f;
                m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;
            }

            if (data.step.warmStarting)
            {
                // Scale the impulse to support a variable time step.
                m_impulse *= data.step.dtRatio;

                Vector2 P = m_impulse * m_u;
                vA -= m_invMassA * P;
                wA -= m_invIA * Vectex.Cross(m_rA, P);
                vB += m_invMassB * P;
                wB += m_invIB * Vectex.Cross(m_rB, P);
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

            // Cdot = dot(u, v + cross(w, r))
            Vector2 vpA = vA + Vectex.Cross(wA, m_rA);
            Vector2 vpB = vB + Vectex.Cross(wB, m_rB);
            float Cdot = Vector2.Dot(m_u, vpB - vpA);

            float impulse = -m_mass * (Cdot + m_bias + m_gamma * m_impulse);
            m_impulse += impulse;

            Vector2 P = impulse * m_u;
            vA -= m_invMassA * P;
            wA -= m_invIA * Vectex.Cross(m_rA, P);
            vB += m_invMassB * P;
            wB += m_invIB * Vectex.Cross(m_rB, P);

            data.velocities[m_indexA].v = vA;
            data.velocities[m_indexA].w = wA;
            data.velocities[m_indexB].v = vB;
            data.velocities[m_indexB].w = wB;
        }

        internal override bool SolvePositionConstraints(in SolverData data)
        {
            if (Stiffness > 0.0f)
            //There is no position correction for soft distance constraints.
            {
                return true;
            }

            Vector2 cA = data.positions[m_indexA].c;
            float aA = data.positions[m_indexA].a;
            Vector2 cB = data.positions[m_indexB].c;
            float aB = data.positions[m_indexB].a;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            Vector2 rA = Math.Mul(qA, m_localAnchorA - m_localCenterA);
            Vector2 rB = Math.Mul(qB, m_localAnchorB - m_localCenterB);
            Vector2 u = cB + rB - cA - rA;

            float length = u.Length();
            u = Vector2.Normalize(u);
            float C = length - m_length;
            C = System.Math.Clamp(C, -Settings.MaxLinearCorrection, Settings.MaxLinearCorrection);

            float impulse = -m_mass * C;
            Vector2 P = impulse * u;

            cA -= m_invMassA * P;
            aA -= m_invIA * Vectex.Cross(rA, P);
            cB += m_invMassB * P;
            aB += m_invIB * Vectex.Cross(rB, P);

            data.positions[m_indexA].c = cA;
            data.positions[m_indexA].a = aA;
            data.positions[m_indexB].c = cB;
            data.positions[m_indexB].a = aB;

            return System.Math.Abs(C) < Settings.LinearSlop;
        }
    }
}