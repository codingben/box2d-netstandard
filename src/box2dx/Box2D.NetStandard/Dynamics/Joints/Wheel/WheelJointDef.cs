using System;
using System.Numerics;
using Box2D.NetStandard.Dynamics.Bodies;

namespace Box2D.NetStandard.Dynamics.Joints.Wheel {
  public class WheelJointDef : JointDef {
    public WheelJointDef() {
      Type = JointType.WheelJoint;
    }
    
    public void Initialize(Body bA,Body bB, in Vector2 anchor, in Vector2 axis, in float frequencyHz = 0f, in float dampingRatio = 0f)
    {
      bodyA        = bA;
      bodyB        = bB;
      localAnchorA = bodyA.GetLocalPoint(anchor);
      localAnchorB = bodyB.GetLocalPoint(anchor);
      localAxisA   = bodyA.GetLocalVector(axis);
      Joint.LinearStiffness(out stiffness, out damping, frequencyHz, dampingRatio, bA, bB);
    }

    [Obsolete("Use Initialize or Joint.LinearStiffness to get stiffness & damping values",true)]
    public float frequencyHz;
    [Obsolete("Use Initialize or Joint.LinearStiffness to get stiffness & damping values",true)]
    public float dampingRatio;
    
    public Vector2 localAnchorA;
    public Vector2 localAnchorB;
    public Vector2 localAxisA;
    public bool    enableLimit;
    public float   lowerTranslation;
    public float   upperTranslation;
    public bool    enableMotor;
    public float   maxMotorTorque;
    public float   motorSpeed;
    public float   stiffness;
    public float   damping;
  }
}