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
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.World;
using Math = Box2D.NetStandard.Common.Math;

namespace Box2D.NetStandard.Dynamics.Joints.Prismatic
{
    /// <summary>
    ///  A prismatic joint. This joint provides one degree of freedom: translation
    ///  along an axis fixed in body1. Relative rotation is prevented. You can
    ///  use a joint limit to restrict the range of motion and a joint motor to
    ///  drive the motion or to model joint friction.
    /// </summary>
    public class PrismaticJoint : Joint, IMotorisedJoint
    {
        private readonly Vector2 m_localYAxisA;
        internal readonly float m_referenceAngle;
        private float m_a1;
        private float m_a2;
        private float m_axialMass;
        private Vector2 m_axis, m_perp;
        private Vector2 m_impulse;
        private int m_indexA;
        private int m_indexB;
        private float m_invIA;
        private float m_invIB;
        private float m_invMassA;
        private float m_invMassB;
        private Matrix3x2 m_k;
        internal Vector2 m_localAnchorA;
        internal Vector2 m_localAnchorB;
        private Vector2 m_localCenterA;
        private Vector2 m_localCenterB;
        internal Vector2 m_localXAxisA;
        private float m_lowerImpulse;
        private float m_maxMotorForce;
        private float m_motorSpeed;
        private float m_s1;
        private float m_s2;
        private float m_translation;
        private float m_upperImpulse;

        public PrismaticJoint(PrismaticJointDef def)
            : base(def)
        {
            m_localAnchorA = def.localAnchorA;
            m_localAnchorB = def.localAnchorB;
            m_localXAxisA = Vector2.Normalize(def.localAxisA);
            m_localYAxisA = Vectex.Cross(1.0f, m_localXAxisA);
            m_referenceAngle = def.referenceAngle;

            m_impulse = Vector2.Zero;
            m_axialMass = 0.0f;
            MotorForce = 0.0f;
            m_lowerImpulse = 0.0f;
            m_upperImpulse = 0.0f;

            LowerLimit = def.lowerTranslation;
            UpperLimit = def.upperTranslation;

            m_maxMotorForce = def.maxMotorForce;
            m_motorSpeed = def.motorSpeed;
            IsLimitEnabled = def.enableLimit;
            IsMotorEnabled = def.enableMotor;

            m_translation = 0.0f;
            m_axis = Vector2.Zero;
            m_perp = Vector2.Zero;
        }

        public override Vector2 GetAnchorA => m_bodyA.GetWorldPoint(m_localAnchorA);

        public override Vector2 GetAnchorB => m_bodyB.GetWorldPoint(m_localAnchorB);

        /// <summary>
        ///  Is the joint limit enabled?
        /// </summary>
        public bool IsLimitEnabled
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
            private set;
        }

        /// <summary>
        ///  Get the lower joint limit, usually in meters.
        /// </summary>
        public float LowerLimit
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
            private set;
        }

        /// <summary>
        ///  Get the upper joint limit, usually in meters.
        /// </summary>
        public float UpperLimit
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
            private set;
        }

        /// <summary>
        ///  Is the joint motor enabled?
        /// </summary>
        public bool IsMotorEnabled
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
            private set;
        }

        /// <summary>
        ///  Get the current motor force, usually in N.
        /// </summary>
        public float MotorForce
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
            private set;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetMotorSpeed(float speed)
        {
            m_bodyA.SetAwake(true);
            m_bodyB.SetAwake(true);
            m_motorSpeed = speed;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float GetMotorSpeed() => m_motorSpeed;

        /// <summary>
        ///  Get\Set the motor speed, usually in meters per second.
        /// </summary>
        public float MotorSpeed
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => m_motorSpeed;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set => SetMotorSpeed(value);
        }

        public override Vector2 GetReactionForce(float inv_dt) =>
            inv_dt * (m_impulse.X * m_perp + (MotorForce + m_lowerImpulse + m_upperImpulse) * m_axis);

        public override float GetReactionTorque(float invDt) => invDt * m_impulse.Y;

        /// <summary>
        ///  Get the current joint translation, usually in meters.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float GetJointTranslation()
        {
            Body b1 = m_bodyA;
            Body b2 = m_bodyB;

            Vector2 p1 = b1.GetWorldPoint(m_localAnchorA);
            Vector2 p2 = b2.GetWorldPoint(m_localAnchorB);
            Vector2 d = p2 - p1;
            Vector2 axis = b1.GetWorldVector(m_localXAxisA);

            return Vector2.Dot(d, axis);
        }

        /// <summary>
        ///  Get the current joint translation speed, usually in meters per second.
        /// </summary>
        public float JointSpeed()
        {
            Body b1 = m_bodyA;
            Body b2 = m_bodyB;

            var r1 = Vector2.Transform(m_localAnchorA - b1.GetLocalCenter(), b1.GetTransform().q);
            var r2 = Vector2.Transform(m_localAnchorB - b2.GetLocalCenter(), b2.GetTransform().q);
            Vector2 p1 = b1.m_sweep.c + r1;
            Vector2 p2 = b2.m_sweep.c + r2;
            Vector2 d = p2 - p1;
            Vector2 axis = b1.GetWorldVector(m_localXAxisA);

            Vector2 v1 = b1.m_linearVelocity;
            Vector2 v2 = b2.m_linearVelocity;
            float w1 = b1.m_angularVelocity;
            float w2 = b2.m_angularVelocity;

            return Vector2.Dot(d, Vectex.Cross(w1, axis)) +
                   Vector2.Dot(axis, v2 + Vectex.Cross(w2, r2) - v1 - Vectex.Cross(w1, r1));
        }

        /// <summary>
        ///  Enable/disable the joint limit.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnableLimit(bool flag)
        {
            m_bodyA.SetAwake(true);
            m_bodyB.SetAwake(true);
            IsLimitEnabled = flag;
        }

        /// <summary>
        ///  Set the joint limits, usually in meters.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetLimits(float lower, float upper)
        {
            //Debug.Assert(lower <= upper);
            m_bodyA.SetAwake(true);
            m_bodyB.SetAwake(true);
            LowerLimit = lower;
            UpperLimit = upper;
        }

        /// <summary>
        ///  Enable/disable the joint motor.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnableMotor(bool flag)
        {
            m_bodyA.SetAwake(true);
            m_bodyB.SetAwake(true);
            IsMotorEnabled = flag;
        }

        /// <summary>
        ///  Set the maximum motor force, usually in N.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetMaxMotorForce(float force)
        {
            m_bodyA.SetAwake(true);
            m_bodyB.SetAwake(true);
            m_maxMotorForce = Settings.FORCE_SCALE(1.0f) * force;
        }

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

            // Compute the effective masses.
            Vector2 rA = Math.Mul(qA, m_localAnchorA - m_localCenterA);
            Vector2 rB = Math.Mul(qB, m_localAnchorB - m_localCenterB);
            Vector2 d = cB - cA + rB - rA;

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            // Compute motor Jacobian and effective mass.
            {
                m_axis = Math.Mul(qA, m_localXAxisA);
                m_a1 = Vectex.Cross(d + rA, m_axis);
                m_a2 = Vectex.Cross(rB, m_axis);

                m_axialMass = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;
                if (m_axialMass > 0.0f)
                {
                    m_axialMass = 1.0f / m_axialMass;
                }
            }

            // Prismatic constraint.
            {
                m_perp = Math.Mul(qA, m_localYAxisA);

                m_s1 = Vectex.Cross(d + rA, m_perp);
                m_s2 = Vectex.Cross(rB, m_perp);

                float k11 = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2;
                float k12 = iA * m_s1 + iB * m_s2;
                float k22 = iA + iB;
                if (k22 == 0.0f)
                // For bodies with fixed rotation.
                {
                    k22 = 1.0f;
                }

                m_k = new Matrix3x2(k11, k12, k12, k22, 0, 0);
            }

            if (IsLimitEnabled)
            {
                m_translation = Vector2.Dot(m_axis, d);
            }
            else
            {
                m_lowerImpulse = 0.0f;
                m_upperImpulse = 0.0f;
            }

            if (IsMotorEnabled == false)
            {
                MotorForce = 0.0f;
            }

            if (data.step.warmStarting)
            {
                // Account for variable time step.
                m_impulse *= data.step.dtRatio;
                MotorForce *= data.step.dtRatio;
                m_lowerImpulse *= data.step.dtRatio;
                m_upperImpulse *= data.step.dtRatio;

                float axialImpulse = MotorForce + m_lowerImpulse - m_upperImpulse;
                Vector2 P = m_impulse.X * m_perp + axialImpulse * m_axis;
                float LA = m_impulse.X * m_s1 + m_impulse.Y + axialImpulse * m_a1;
                float LB = m_impulse.X * m_s2 + m_impulse.Y + axialImpulse * m_a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }
            else
            {
                m_impulse = Vector2.Zero;
                MotorForce = 0.0f;
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
            Vector2 vA = data.velocities[m_indexA].v;
            float wA = data.velocities[m_indexA].w;
            Vector2 vB = data.velocities[m_indexB].v;
            float wB = data.velocities[m_indexB].w;

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            // Solve linear motor constraint
            if (IsMotorEnabled)
            {
                float Cdot = Vector2.Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
                float impulse = m_axialMass * (m_motorSpeed - Cdot);
                float oldImpulse = MotorForce;
                float maxImpulse = data.step.dt * m_maxMotorForce;
                MotorForce = System.Math.Clamp(MotorForce + impulse, -maxImpulse, maxImpulse);
                impulse = MotorForce - oldImpulse;

                Vector2 P = impulse * m_axis;
                float LA = impulse * m_a1;
                float LB = impulse * m_a2;

                vA -= mA * P;
                wA -= iA * LA;
                vB += mB * P;
                wB += iB * LB;
            }

            if (IsLimitEnabled)
            {
                // Lower limit
                {
                    float C = m_translation - LowerLimit;
                    float Cdot = Vector2.Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
                    float impulse = -m_axialMass * (Cdot + MathF.Max(C, 0.0f) * data.step.inv_dt);
                    float oldImpulse = m_lowerImpulse;
                    m_lowerImpulse = MathF.Max(m_lowerImpulse + impulse, 0.0f);
                    impulse = m_lowerImpulse - oldImpulse;

                    Vector2 P = impulse * m_axis;
                    float LA = impulse * m_a1;
                    float LB = impulse * m_a2;

                    vA -= mA * P;
                    wA -= iA * LA;
                    vB += mB * P;
                    wB += iB * LB;
                }

                // Upper limit
                // Note: signs are flipped to keep C positive when the constraint is satisfied.
                // This also keeps the impulse positive when the limit is active.
                {
                    float C = UpperLimit - m_translation;
                    float Cdot = Vector2.Dot(m_axis, vA - vB) + m_a1 * wA - m_a2 * wB;
                    float impulse = -m_axialMass * (Cdot + MathF.Max(C, 0.0f) * data.step.inv_dt);
                    float oldImpulse = m_upperImpulse;
                    m_upperImpulse = MathF.Max(m_upperImpulse + impulse, 0.0f);
                    impulse = m_upperImpulse - oldImpulse;

                    Vector2 P = impulse * m_axis;
                    float LA = impulse * m_a1;
                    float LB = impulse * m_a2;

                    vA += mA * P;
                    wA += iA * LA;
                    vB -= mB * P;
                    wB -= iB * LB;
                }
            }

            // Solve the prismatic constraint in block form.
            {
                var Cdot = new Vector2();
                Cdot.X = Vector2.Dot(m_perp, vB - vA) + m_s2 * wB - m_s1 * wA;
                Cdot.Y = wB - wA;

                Vector2 df = m_k.Solve(-Cdot);
                m_impulse += df;

                Vector2 P = df.X * m_perp;
                float LA = df.X * m_s1 + df.Y;
                float LB = df.X * m_s2 + df.Y;

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

            Rot qA = new Rot(aA), qB = new Rot(aB);

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            // Compute fresh Jacobians
            Vector2 rA = Math.Mul(qA, m_localAnchorA - m_localCenterA);
            Vector2 rB = Math.Mul(qB, m_localAnchorB - m_localCenterB);
            Vector2 d = cB + rB - cA - rA;

            Vector2 axis = Math.Mul(qA, m_localXAxisA);
            float a1 = Vectex.Cross(d + rA, axis);
            float a2 = Vectex.Cross(rB, axis);
            Vector2 perp = Math.Mul(qA, m_localYAxisA);

            float s1 = Vectex.Cross(d + rA, perp);
            float s2 = Vectex.Cross(rB, perp);

            Vector3 impulse;
            var C1 = new Vector2();
            C1.X = Vector2.Dot(perp, d);
            C1.Y = aB - aA - m_referenceAngle;

            float linearError = MathF.Abs(C1.X);
            float angularError = MathF.Abs(C1.Y);

            var active = false;
            var C2 = 0.0f;
            if (IsLimitEnabled)
            {
                float translation = Vector2.Dot(axis, d);
                if (MathF.Abs(UpperLimit - LowerLimit) < 2.0f * Settings.LinearSlop)
                {
                    C2 = translation;
                    linearError = MathF.Max(linearError, MathF.Abs(translation));
                    active = true;
                }
                else if (translation <= LowerLimit)
                {
                    C2 = MathF.Min(translation - LowerLimit, 0.0f);
                    linearError = MathF.Max(linearError, LowerLimit - translation);
                    active = true;
                }
                else if (translation >= UpperLimit)
                {
                    C2 = MathF.Max(translation - UpperLimit, 0.0f);
                    linearError = MathF.Max(linearError, translation - UpperLimit);
                    active = true;
                }
            }

            if (active)
            {
                float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                float k12 = iA * s1 + iB * s2;
                float k13 = iA * s1 * a1 + iB * s2 * a2;
                float k22 = iA + iB;
                if (k22 == 0.0f)
                // For fixed rotation
                {
                    k22 = 1.0f;
                }

                float k23 = iA * a1 + iB * a2;
                float k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

                var K = new Mat33();
                K.ex = new Vector3(k11, k12, k13);
                K.ey = new Vector3(k12, k22, k23);
                K.ez = new Vector3(k13, k23, k33);

                var C = new Vector3();
                C.X = C1.X;
                C.Y = C1.Y;
                C.Z = C2;

                impulse = K.Solve33(-C);
            }
            else
            {
                float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                float k12 = iA * s1 + iB * s2;
                float k22 = iA + iB;
                if (k22 == 0.0f)
                {
                    k22 = 1.0f;
                }

                //Mat22 K = new Mat22();
                var K = new Matrix3x2(k11, k12, k12, k22, 0, 0);
                // K.ex=new Vector2(k11, k12);
                // K.ey=new Vector2(k12, k22);

                Vector2 impulse1 = K.Solve(-C1);
                impulse.X = impulse1.X;
                impulse.Y = impulse1.Y;
                impulse.Z = 0.0f;
            }

            Vector2 P = impulse.X * perp + impulse.Z * axis;
            float LA = impulse.X * s1 + impulse.Y + impulse.Z * a1;
            float LB = impulse.X * s2 + impulse.Y + impulse.Z * a2;

            cA -= mA * P;
            aA -= iA * LA;
            cB += mB * P;
            aB += iB * LB;

            data.positions[m_indexA].c = cA;
            data.positions[m_indexA].a = aA;
            data.positions[m_indexB].c = cB;
            data.positions[m_indexB].a = aB;

            return linearError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
        }
    }
}