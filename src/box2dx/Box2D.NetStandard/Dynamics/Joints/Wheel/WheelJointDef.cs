using System.Numerics;

namespace Box2D.NetStandard.Dynamics.Joints.Wheel {
  public class WheelJointDef : JointDef {
    public WheelJointDef() {
      Type             = JointType.WheelJoint;
      LocalAnchor1     = Vector2.Zero;
      LocalAnchor2     = Vector2.Zero;
      LocalAxisA       = Vector2.Zero;
      EnableLimit      = false;
      LowerTranslation = 0f;
      UpperTranslation = 0f;
      EnableMotor      = false;
      MaxMotorTorque   = 0f;
      MotorSpeed       = 0f;
      Stiffness        = 0f;
      Damping          = 0f;
    }

    public Vector2 LocalAnchor1;
    public Vector2 LocalAnchor2;
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