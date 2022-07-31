using MathF = System.Math;
using Float = System.Double;

namespace Box2D.NetStandard.Dynamics.Joints.Gear
{
    /// <summary>
    ///  Gear joint definition. This definition requires two existing
    ///  revolute or prismatic joints (any combination will work).
    ///  The provided joints must attach a dynamic body to a static body.
    /// </summary>
    public class GearJointDef : JointDef
    {
        /// <summary>
        ///  The first revolute/prismatic joint attached to the gear joint.
        /// </summary>
        public Joint Joint1;

        /// <summary>
        ///  The second revolute/prismatic joint attached to the gear joint.
        /// </summary>
        public Joint Joint2;

        /// <summary>
        ///  The gear ratio.
        ///  @see GearJoint for explanation.
        /// </summary>
        public float Ratio;

        public GearJointDef() => Ratio = 1.0f;
    }
}