using System;
using System.Numerics;
using Box2D.NetStandard.Dynamics.Bodies;

namespace Box2D.NetStandard.Dynamics.Joints.Distance
{
    /// <summary>
    ///  Distance joint definition. This requires defining an
    ///  anchor point on both bodies and the non-zero length of the
    ///  distance joint. The definition uses local anchor points
    ///  so that the initial configuration can violate the constraint
    ///  slightly. This helps when saving and loading a game.
    ///  @warning Do not use a zero or short length.
    /// </summary>
    public class DistanceJointDef : JointDef
    {
        /// <summary>
        ///  The linear damping in N*s/m.
        /// </summary>
        public float damping;

        [Obsolete("Use stiffness and damping instead of frequencyHz and dampingRatio")]
        public float? dampingRatio;

        [Obsolete("Use stiffness and damping instead of frequencyHz and dampingRatio")]
        public float? frequencyHz;

        /// <summary>
        ///  The equilibrium length between the anchor points.
        /// </summary>
        public float length;

        /// <summary>
        ///  The local anchor point relative to body1's origin.
        /// </summary>
        public Vector2 localAnchorA;

        /// <summary>
        ///  The local anchor point relative to body2's origin.
        /// </summary>
        public Vector2 localAnchorB;

        /// <summary>
        ///  The linear stiffness in N/m. A value of 0 disables softness.
        /// </summary>
        public float stiffness;

        public DistanceJointDef() => length = 1.0f;

        /// <summary>
        ///  Initialize the bodies, anchors, and length using the world anchors.
        /// </summary>
        public void Initialize(Body bodyA, Body bodyB, Vector2 anchor1, Vector2 anchor2, float frequencyHz = 0f,
            float dampingRatio = 0f)
        {
            this.bodyA = bodyA;
            this.bodyB = bodyB;

            localAnchorA = bodyA.GetLocalPoint(anchor1);
            localAnchorB = bodyB.GetLocalPoint(anchor2);

            Vector2 d = anchor2 - anchor1;
            length = d.Length();

            Joint.LinearStiffness(out stiffness, out damping, frequencyHz, dampingRatio, bodyA, bodyB);
        }
    }
}