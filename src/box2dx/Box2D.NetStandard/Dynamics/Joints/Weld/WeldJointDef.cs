using System.Numerics;

namespace Box2D.NetStandard.Dynamics.Joints.Weld {
  public class WeldJointDef : JointDef {
    public WeldJointDef()
    {
      Type = JointType.WeldJoint;
    }

    public Vector2 localAnchorA;
    public Vector2 localAnchorB;
    public float referenceAngle;
    /// <summary>
    /// The rotational stiffness in N*m
    /// Disable softness with a value of 0
    /// </summary>
    public float stiffness;
    /// <summary>
    /// The rotational damping in N*m*s
    /// </summary>
    public float damping;
  }
}