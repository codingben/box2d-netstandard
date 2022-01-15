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
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.World;
using Math = Box2D.NetStandard.Common.Math;

namespace Box2D.NetStandard.Dynamics.Joints.Revolute
{
    /// <summary>
    ///  A revolute joint constrains to bodies to share a common point while they
    ///  are free to rotate about the point. The relative rotation about the shared
    ///  point is the joint angle. You can limit the relative rotation with
    ///  a joint limit that specifies a lower and upper angle. You can use a motor
    ///  to drive the relative rotation about the shared point. A maximum motor torque
    ///  is provided so that infinite forces are not generated.
    /// </summary>
    public class RevoluteJoint : Joint, IMotorisedJoint
    {
        internal readonly float m_referenceAngle;
        private float m_angle;
        private float m_axialMass;

        private Vector2 m_impulse;

        private int m_indexA;
        private int m_indexB;
        private float m_invIA;
        private float m_invIB;
        private float m_invMassA;
        private float m_invMassB;
        private Matrix3x2 m_K;
        internal Vector2 m_localAnchorA;
        internal Vector2 m_localAnchorB;
        private Vector2 m_localCenterA;
        private Vector2 m_localCenterB;
        private float m_lowerImpulse;
        private float m_maxMotorTorque;
        private float m_motorSpeed;
        private Vector2 m_rA;
        private Vector2 m_rB;
        private float m_upperImpulse;

        public RevoluteJoint(RevoluteJointDef def)
            : base(def)
        {
            m_localAnchorA = def.localAnchorA;
            m_localAnchorB = def.localAnchorB;
            m_referenceAngle = def.referenceAngle;

            m_impulse = Vector2.Zero;
            m_axialMass = 0.0f;
            MotorTorque = 0.0f;
            m_lowerImpulse = 0.0f;
            m_upperImpulse = 0.0f;

            LowerLimit = def.lowerAngle;
            UpperLimit = def.upperAngle;
            m_maxMotorTorque = def.maxMotorTorque;
            m_motorSpeed = def.motorSpeed;
            IsLimitEnabled = def.enableLimit;
            IsMotorEnabled = def.enableMotor;

            m_angle = 0.0f;
        }

        public override Vector2 GetAnchorA => m_bodyA.GetWorldPoint(m_localAnchorA);

        public override Vector2 GetAnchorB => m_bodyB.GetWorldPoint(m_localAnchorB);

        /// <summary>
        ///  Get the current joint angle in radians.
        /// </summary>
        public float JointAngle
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                Body b1 = m_bodyA;
                Body b2 = m_bodyB;
                return b2.m_sweep.a - b1.m_sweep.a - m_referenceAngle;
            }
        }


        /// <summary>
        ///  Get the current joint angle speed in radians per second.
        /// </summary>
        public float JointSpeed
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                Body b1 = m_bodyA;
                Body b2 = m_bodyB;
                return b2.m_angularVelocity - b1.m_angularVelocity;
            }
        }

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
        ///  Get the lower joint limit in radians.
        /// </summary>
        public float LowerLimit
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
            private set;
        }

        /// <summary>
        ///  Get the upper joint limit in radians.
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
        ///  Get the current motor torque, usually in N-m.
        /// </summary>
        public float MotorTorque
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
            private set;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float GetMotorSpeed() => m_motorSpeed;

        /// <summary>
        ///  Get/Set the motor speed in radians per second.
        /// </summary>
        public float MotorSpeed
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => m_motorSpeed;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set => SetMotorSpeed(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetMotorSpeed(float speed)
        {
            m_motorSpeed = speed;
            m_bodyA.SetAwake(true);
            m_bodyB.SetAwake(true);
            m_motorSpeed = speed;
        }

        public override Vector2 GetReactionForce(float invDt) => invDt * new Vector2(m_impulse.X, m_impulse.Y);

        public override float GetReactionTorque(float inv_dt) => inv_dt * (m_lowerImpulse + m_upperImpulse);

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
        ///  Set the joint limits in radians.
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
        ///  Set the maximum motor torque, usually in N-m.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetMaxMotorTorque(float torque)
        {
            m_bodyA.SetAwake(true);
            m_bodyB.SetAwake(true);
            m_maxMotorTorque = torque;
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

            float aA = data.positions[m_indexA].a;
            Vector2 vA = data.velocities[m_indexA].v;
            float wA = data.velocities[m_indexA].w;

            float aB = data.positions[m_indexB].a;
            Vector2 vB = data.velocities[m_indexB].v;
            float wB = data.velocities[m_indexB].w;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            m_rA = Math.Mul(qA, m_localAnchorA - m_localCenterA);
            m_rB = Math.Mul(qB, m_localAnchorB - m_localCenterB);

            // J = [-I -r1_skew I r2_skew]
            //     [ 0       -1 0       1]
            // r_skew = [-ry; rx]

            // Matlab
            // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
            //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            // ex.x = M11 ey.x = M12
            // ex.y = M21 ey.y = M22

            m_K.M11 = mA + mB + m_rA.Y * m_rA.Y * iA + m_rB.Y * m_rB.Y * iB;
            m_K.M12 = -m_rA.Y * m_rA.X * iA - m_rB.Y * m_rB.X * iB;
            m_K.M21 = m_K.M12;
            m_K.M22 = mA + mB + m_rA.X * m_rA.X * iA + m_rB.X * m_rB.X * iB;

            m_axialMass = iA + iB;
            var fixedRotation = false;
            if (m_axialMass > 0f)
            {
                m_axialMass = 1f / m_axialMass;
                fixedRotation = true;
            }

            if (IsMotorEnabled == false || fixedRotation)
            {
                MotorTorque = 0.0f;
            }
            else
            {
                fixedRotation = true;
            }

            m_angle = aB - aA - m_referenceAngle;
            if (IsLimitEnabled == false || fixedRotation)
            {
                m_lowerImpulse = 0.0f;
                m_upperImpulse = 0.0f;
            }

            if (IsMotorEnabled == false || fixedRotation)
            {
                MotorTorque = 0.0f;
            }

            if (data.step.warmStarting)
            {
                // Scale impulses to support a variable time step.
                m_impulse *= data.step.dtRatio;
                MotorTorque *= data.step.dtRatio;
                m_lowerImpulse *= data.step.dtRatio;
                m_upperImpulse *= data.step.dtRatio;

                float axialImpulse = MotorTorque + m_lowerImpulse - m_upperImpulse;

                var P = new Vector2(m_impulse.X, m_impulse.Y);

                vA -= mA * P;
                wA -= iA * (Vectex.Cross(m_rA, P) + axialImpulse);

                vB += mB * P;
                wB += iB * (Vectex.Cross(m_rB, P) + axialImpulse);
            }
            else
            {
                m_impulse = Vector2.Zero;
                MotorTorque = 0.0f;
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

            bool fixedRotation = iA + iB == 0.0f;

            // Solve motor constraint.
            if (IsMotorEnabled && fixedRotation == false)
            {
                float Cdot = wB - wA - m_motorSpeed;
                float impulse = -m_axialMass * Cdot;
                float oldImpulse = MotorTorque;
                float maxImpulse = data.step.dt * m_maxMotorTorque;
                MotorTorque = System.Math.Clamp(MotorTorque + impulse, -maxImpulse, maxImpulse);
                impulse = MotorTorque - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Solve limit constraint.
            if (IsLimitEnabled && fixedRotation == false)
            {
                // Lower limit
                {
                    float C = m_angle - LowerLimit;
                    float Cdot = wB - wA;
                    float impulse = -m_axialMass * (Cdot + MathF.Max(C, 0.0f) * data.step.inv_dt);
                    float oldImpulse = m_lowerImpulse;
                    m_lowerImpulse = MathF.Max(m_lowerImpulse + impulse, 0.0f);
                    impulse = m_lowerImpulse - oldImpulse;

                    wA -= iA * impulse;
                    wB += iB * impulse;
                }

                // Upper limit
                // Note: signs are flipped to keep C positive when the constraint is satisfied.
                // This also keeps the impulse positive when the limit is active.
                {
                    float C = UpperLimit - m_angle;
                    float Cdot = wA - wB;
                    float impulse = -m_axialMass * (Cdot + MathF.Max(C, 0.0f) * data.step.inv_dt);
                    float oldImpulse = m_upperImpulse;
                    m_upperImpulse = MathF.Max(m_upperImpulse + impulse, 0.0f);
                    impulse = m_upperImpulse - oldImpulse;

                    wA += iA * impulse;
                    wB -= iB * impulse;
                }
            }

            // Solve point-to-point constraint
            {
                Vector2 Cdot = vB + Vectex.Cross(wB, m_rB) - vA - Vectex.Cross(wA, m_rA);
                Vector2 impulse = m_K.Solve(-Cdot);

                m_impulse.X += impulse.X;
                m_impulse.Y += impulse.Y;
                vA -= mA * impulse;
                wA -= iA * Vectex.Cross(m_rA, impulse);
                vB += mB * impulse;
                wB += iB * Vectex.Cross(m_rB, impulse);
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

            var angularError = 0.0f;
            var positionError = 0.0f;

            bool fixedRotation = m_invIA + m_invIB == 0.0f;

            // Solve angular limit constraint.
            if (IsLimitEnabled && fixedRotation == false)
            {
                float angle = aB - aA - m_referenceAngle;
                var C = 0.0f;

                if (MathF.Abs(UpperLimit - LowerLimit) < 2.0f * Settings.AngularSlop)
                // Prevent large angular corrections
                {
                    C = System.Math.Clamp(angle - LowerLimit, -Settings.MaxAngularCorrection, Settings.MaxAngularCorrection);
                }
                else if (angle < LowerLimit)
                // Prevent large angular corrections and allow some slop.
                {
                    C = System.Math.Clamp(angle - LowerLimit + Settings.AngularSlop, -Settings.MaxAngularCorrection, 0.0f);
                }
                else if (angle >= UpperLimit)
                // Prevent large angular corrections and allow some slop.
                {
                    C = System.Math.Clamp(angle - UpperLimit - Settings.AngularSlop, 0.0f, Settings.MaxAngularCorrection);
                }

                float limitImpulse = -m_axialMass * C;
                aA -= m_invIA * limitImpulse;
                aB += m_invIB * limitImpulse;
                angularError = MathF.Abs(C);
            }

            // Solve point-to-point constraint.
            {
                qA.Set(aA);
                qB.Set(aB);
                Vector2 rA = Math.Mul(qA, m_localAnchorA - m_localCenterA);
                Vector2 rB = Math.Mul(qB, m_localAnchorB - m_localCenterB);

                Vector2 C = cB + rB - cA - rA;
                positionError = C.Length();

                float mA = m_invMassA, mB = m_invMassB;
                float iA = m_invIA, iB = m_invIB;

                var K = new Matrix3x2();
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

            data.positions[m_indexA].c = cA;
            data.positions[m_indexA].a = aA;
            data.positions[m_indexB].c = cB;
            data.positions[m_indexB].a = aB;

            return positionError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
        }
    }
}