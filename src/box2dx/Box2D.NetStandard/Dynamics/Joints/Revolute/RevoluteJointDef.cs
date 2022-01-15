using System.Numerics;
using Box2D.NetStandard.Dynamics.Bodies;

namespace Box2D.NetStandard.Dynamics.Joints.Revolute
{
    /// <summary>
    ///  Revolute joint definition. This requires defining an
    ///  anchor point where the bodies are joined. The definition
    ///  uses local anchor points so that the initial configuration
    ///  can violate the constraint slightly. You also need to
    ///  specify the initial relative angle for joint limits. This
    ///  helps when saving and loading a game.
    ///  The local anchor points are measured from the body's origin
    ///  rather than the center of mass because:
    ///  1. you might not know where the center of mass will be.
    ///  2. if you add/remove shapes from a body and recompute the mass,
    ///  the joints will be broken.
    /// </summary>
    public class RevoluteJointDef : JointDef
    {
        /// <summary>
        ///  A flag to enable joint limits.
        /// </summary>
        public bool enableLimit;

        /// <summary>
        ///  A flag to enable the joint motor.
        /// </summary>
        public bool enableMotor;

        /// <summary>
        ///  The local anchor point relative to body1's origin.
        /// </summary>
        public Vector2 localAnchorA;

        /// <summary>
        ///  The local anchor point relative to body2's origin.
        /// </summary>
        public Vector2 localAnchorB;

        /// <summary>
        ///  The lower angle for the joint limit (radians).
        /// </summary>
        public float lowerAngle;

        /// <summary>
        ///  The maximum motor torque used to achieve the desired motor speed.
        ///  Usually in N-m.
        /// </summary>
        public float maxMotorTorque;

        /// <summary>
        ///  The desired motor speed. Usually in radians per second.
        /// </summary>
        public float motorSpeed;

        /// <summary>
        ///  The body2 angle minus body1 angle in the reference state (radians).
        /// </summary>
        public float referenceAngle;

        /// <summary>
        ///  The upper angle for the joint limit (radians).
        /// </summary>
        public float upperAngle;

        /// <summary>
        ///  Initialize the bodies, anchors, and reference angle using the world
        ///  anchor.
        /// </summary>
        public void Initialize(Body body1, Body body2, Vector2 anchor)
        {
            bodyA = body1;
            bodyB = body2;
            localAnchorA = body1.GetLocalPoint(anchor);
            localAnchorB = body2.GetLocalPoint(anchor);
            referenceAngle = body2.GetAngle() - body1.GetAngle();
        }
    }
}