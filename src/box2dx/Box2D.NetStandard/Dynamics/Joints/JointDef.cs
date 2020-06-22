using Box2D.NetStandard.Dynamics.Bodies;

namespace Box2D.NetStandard.Dynamics.Joints {
  /// <summary>
  /// Joint definitions are used to construct joints.
  /// </summary>
  public class JointDef
  {
    public JointDef()
    {
      Type             = JointType.UnknownJoint;
      UserData         = null;
      bodyA            = null;
      bodyB            = null;
      collideConnected = false;
    }

    /// <summary>
    /// The joint type is set automatically for concrete joint types.
    /// </summary>
    public JointType Type;

    /// <summary>
    /// Use this to attach application specific data to your joints.
    /// </summary>
    public object UserData;

    /// <summary>
    /// The first attached body.
    /// </summary>
    public Body bodyA;

    /// <summary>
    /// The second attached body.
    /// </summary>
    public Body bodyB;

    /// <summary>
    /// Set this flag to true if the attached bodies should collide.
    /// </summary>
    public bool collideConnected;
  }
}