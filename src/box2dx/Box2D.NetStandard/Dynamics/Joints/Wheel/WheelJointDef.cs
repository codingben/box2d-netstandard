using System.Numerics;

namespace Box2D.NetStandard.Dynamics.Joints.Wheel {
  public class WheelJointDef : JointDef {
    public WheelJointDef() {
      Type = JointType.WheelJoint;
    }

    public Vector2 LocalAnchorA;
    public Vector2 LocalAnchorB;
    public Vector2 LocalAxisA;
    public bool    EnableLimit;
    public float   LowerTranslation;
    public float   UpperTranslation;
    public bool    EnableMotor;
    public float   MaxMotorTorque;
    public float   MotorSpeed;
    public float   Stiffness;
    public float   Damping;
  }
}