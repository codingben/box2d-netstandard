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
    public float frequencyHz;
    public float dampingRatio;
  }
}