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
      BodyA            = null;
      BodyB            = null;
      CollideConnected = false;
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
    public Body BodyA;

    /// <summary>
    /// The second attached body.
    /// </summary>
    public Body BodyB;

    /// <summary>
    /// Set this flag to true if the attached bodies should collide.
    /// </summary>
    public bool CollideConnected;
  }
}