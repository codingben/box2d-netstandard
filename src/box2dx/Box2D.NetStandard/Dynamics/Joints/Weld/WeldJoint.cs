using System;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.World;
using b2Vec2 = System.Numerics.Vector2;
using b2Vec3 = System.Numerics.Vector3;
using Math = Box2D.NetStandard.Common.Math;

namespace Box2D.NetStandard.Dynamics.Joints.Weld
{
    public class WeldJoint : Joint
    {
        private float m_bias;
        private float m_gamma;
        private b2Vec3 m_impulse;

        private int m_indexA;
        private int m_indexB;
        private float m_invIA;
        private float m_invIB;
        private float m_invMassA;
        private float m_invMassB;

        private b2Vec2 m_localAnchorA;
        private b2Vec2 m_localAnchorB;
        private b2Vec2 m_localCenterA;
        private b2Vec2 m_localCenterB;
        private Mat33 m_mass;
        private b2Vec2 m_rA;
        private b2Vec2 m_rB;
        private float m_referenceAngle;

        public WeldJoint(WeldJointDef def) : base(def)
        {
            m_localAnchorA = def.localAnchorA;
            m_localAnchorB = def.localAnchorB;
            m_referenceAngle = def.referenceAngle;
            Stiffness = def.stiffness;
            Damping = def.damping;
        }

        public override b2Vec2 GetAnchorA => m_bodyA.GetWorldPoint(m_localAnchorA);
        public override b2Vec2 GetAnchorB => m_bodyB.GetWorldPoint(m_localAnchorB);

        public float Stiffness
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set;
        }

        public float Damping
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set;
        }

        public override b2Vec2 GetReactionForce(float inv_dt) => inv_dt * new b2Vec2(m_impulse.X, m_impulse.Y);

        public override float GetReactionTorque(float inv_dt) => inv_dt * m_impulse.Z;


        public void Initialize(Body bA, Body bB, in b2Vec2 anchor)
        {
            m_bodyA = bA;
            m_bodyB = bB;
            m_localAnchorA = m_bodyA.GetLocalPoint(anchor);
            m_localAnchorB = m_bodyB.GetLocalPoint(anchor);
            m_referenceAngle = m_bodyB.GetAngle() - m_bodyA.GetAngle();
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
            b2Vec2 vA = data.velocities[m_indexA].v;
            float wA = data.velocities[m_indexA].w;

            float aB = data.positions[m_indexB].a;
            b2Vec2 vB = data.velocities[m_indexB].v;
            float wB = data.velocities[m_indexB].w;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            m_rA = Math.Mul(qA, m_localAnchorA - m_localCenterA);
            m_rB = Math.Mul(qB, m_localAnchorB - m_localCenterB);

            // J = [-I -r1_skew I r2_skew]
            //     [ 0       -1 0       1]
            // r_skew = [-ry; rx]

            // Matlab
            // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
            //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
            //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            Mat33 K;
            K.ex.X = mA + mB + m_rA.Y * m_rA.Y * iA + m_rB.Y * m_rB.Y * iB;
            K.ey.X = -m_rA.Y * m_rA.X * iA - m_rB.Y * m_rB.X * iB;
            K.ez.X = -m_rA.Y * iA - m_rB.Y * iB;
            K.ex.Y = K.ey.X;
            K.ey.Y = mA + mB + m_rA.X * m_rA.X * iA + m_rB.X * m_rB.X * iB;
            K.ez.Y = m_rA.X * iA + m_rB.X * iB;
            K.ex.Z = K.ez.X;
            K.ey.Z = K.ez.Y;
            K.ez.Z = iA + iB;

            if (Stiffness > 0.0f)
            {
                m_mass = K.GetInverse22(m_mass);

                float invM = iA + iB;

                float C = aB - aA - m_referenceAngle;

                float d = Damping;

                float k = Stiffness;

                // magic formulas
                float h = data.step.dt;
                m_gamma = h * (d + h * k);
                m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
                m_bias = C * h * k * m_gamma;

                invM += m_gamma;
                m_mass.ez.Z = invM != 0.0f ? 1.0f / invM : 0.0f;
            }
            else if (K.ez.Z == 0.0f)
            {
                m_mass = K.GetInverse22(m_mass);
                m_gamma = 0.0f;
                m_bias = 0.0f;
            }
            else
            {
                m_mass = K.GetSymInverse33(m_mass);
                m_gamma = 0.0f;
                m_bias = 0.0f;
            }

            if (data.step.warmStarting)
            {
                // Scale impulses to support a variable time step.
                m_impulse *= data.step.dtRatio;

                var P = new b2Vec2(m_impulse.X, m_impulse.Y);

                vA -= mA * P;
                wA -= iA * (Vectex.Cross(m_rA, P) + m_impulse.Z);

                vB += mB * P;
                wB += iB * (Vectex.Cross(m_rB, P) + m_impulse.Z);
            }
            else
            {
                m_impulse = b2Vec3.Zero;
            }

            data.velocities[m_indexA].v = vA;
            data.velocities[m_indexA].w = wA;
            data.velocities[m_indexB].v = vB;
            data.velocities[m_indexB].w = wB;
        }

        internal override void SolveVelocityConstraints(in SolverData data)
        {
            b2Vec2 vA = data.velocities[m_indexA].v;
            float wA = data.velocities[m_indexA].w;
            b2Vec2 vB = data.velocities[m_indexB].v;
            float wB = data.velocities[m_indexB].w;
            float mA = m_invMassA, mB = m_invMassB;

            float iA = m_invIA, iB = m_invIB;
            if (Stiffness > 0.0f)
            {
                float Cdot2 = wB - wA;

                float impulse2 = -m_mass.ez.Z * (Cdot2 + m_bias + m_gamma * m_impulse.Z);
                m_impulse.Z += impulse2;

                wA -= iA * impulse2;
                wB += iB * impulse2;

                b2Vec2 Cdot1 = vB + Vectex.Cross(wB, m_rB) - vA - Vectex.Cross(wA, m_rA);

                b2Vec2 impulse1 = -Math.Mul22(m_mass, Cdot1);
                m_impulse.X += impulse1.X;
                m_impulse.Y += impulse1.Y;

                b2Vec2 P = impulse1;

                vA -= mA * P;
                wA -= iA * Vectex.Cross(m_rA, P);

                vB += mB * P;
                wB += iB * Vectex.Cross(m_rB, P);
            }
            else
            {
                b2Vec2 Cdot1 = vB + Vectex.Cross(wB, m_rB) - vA - Vectex.Cross(wA, m_rA);
                float Cdot2 = wB - wA;
                var Cdot = new b2Vec3(Cdot1.X, Cdot1.Y, Cdot2);

                b2Vec3 impulse = -Math.Mul(m_mass, Cdot);
                m_impulse += impulse;

                var P = new b2Vec2(impulse.X, impulse.Y);

                vA -= mA * P;
                wA -= iA * (Vectex.Cross(m_rA, P) + impulse.Z);

                vB += mB * P;
                wB += iB * (Vectex.Cross(m_rB, P) + impulse.Z);
            }

            data.velocities[m_indexA].v = vA;
            data.velocities[m_indexA].w = wA;
            data.velocities[m_indexB].v = vB;
            data.velocities[m_indexB].w = wB;
        }

        internal override bool SolvePositionConstraints(in SolverData data)
        {
            b2Vec2 cA = data.positions[m_indexA].c;
            float aA = data.positions[m_indexA].a;
            b2Vec2 cB = data.positions[m_indexB].c;
            float aB = data.positions[m_indexB].a;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            b2Vec2 rA = Math.Mul(qA, m_localAnchorA - m_localCenterA);
            b2Vec2 rB = Math.Mul(qB, m_localAnchorB - m_localCenterB);

            float positionError, angularError;

            var K = new Mat33();
            K.ex.X = mA + mB + rA.Y * rA.Y * iA + rB.Y * rB.Y * iB;
            K.ey.X = -rA.Y * rA.X * iA - rB.Y * rB.X * iB;
            K.ez.X = -rA.Y * iA - rB.Y * iB;
            K.ex.Y = K.ey.X;
            K.ey.Y = mA + mB + rA.X * rA.X * iA + rB.X * rB.X * iB;
            K.ez.Y = rA.X * iA + rB.X * iB;
            K.ex.Z = K.ez.X;
            K.ey.Z = K.ez.Y;
            K.ez.Z = iA + iB;

            if (Stiffness > 0.0f)
            {
                b2Vec2 C1 = cB + rB - cA - rA;

                positionError = C1.Length();
                angularError = 0.0f;

                b2Vec2 P = -K.Solve22(C1);

                cA -= mA * P;
                aA -= iA * Vectex.Cross(rA, P);

                cB += mB * P;
                aB += iB * Vectex.Cross(rB, P);
            }
            else
            {
                b2Vec2 C1 = cB + rB - cA - rA;
                float C2 = aB - aA - m_referenceAngle;

                positionError = C1.Length();
                angularError = MathF.Abs(C2);

                var C = new b2Vec3(C1.X, C1.Y, C2);

                b2Vec3 impulse;
                if (K.ez.Z > 0.0f)
                {
                    impulse = -K.Solve33(C);
                }
                else
                {
                    b2Vec2 impulse2 = -K.Solve22(C1);
                    impulse = new b2Vec3(impulse2.X, impulse2.Y, 0.0f);
                }

                var P = new b2Vec2(impulse.X, impulse.Y);

                cA -= mA * P;
                aA -= iA * (Vectex.Cross(rA, P) + impulse.Z);

                cB += mB * P;
                aB += iB * (Vectex.Cross(rB, P) + impulse.Z);
            }

            data.positions[m_indexA].c = cA;
            data.positions[m_indexA].a = aA;
            data.positions[m_indexB].c = cB;
            data.positions[m_indexB].a = aB;

            return positionError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
        }
    }
}