using System.Numerics;
using Box2D.NetStandard.Dynamics.Bodies;

namespace Box2D.NetStandard.Dynamics.Joints.Distance
{
  /// <summary>
  /// Distance joint definition. This requires defining an
  /// anchor point on both bodies and the non-zero length of the
  /// distance joint. The definition uses local anchor points
  /// so that the initial configuration can violate the constraint
  /// slightly. This helps when saving and loading a game.
  /// @warning Do not use a zero or short length.
  /// </summary>
  public class DistanceJointDef : JointDef
  {
    public DistanceJointDef()
    {
      Type = JointType.DistanceJoint;
      localAnchorA = new Vector2(0.0f, 0.0f);
      localAnchorB = new Vector2(0.0f, 0.0f);
      length = 1.0f;
      stiffness = 0.0f;
      damping = 0.0f;
    }

    /// <summary>
    /// Initialize the bodies, anchors, and length using the world anchors.
    /// </summary>
    public void Initialize(Body body1, Body body2, Vector2 anchor1, Vector2 anchor2)
    {
      bodyA = body1;
      bodyB = body2;
      localAnchorA = body1.GetLocalPoint(anchor1);
      localAnchorB = body2.GetLocalPoint(anchor2);
      Vector2 d = anchor2 - anchor1;
      length = d.Length();
    }

    /// <summary>
    /// The local anchor point relative to body1's origin.
    /// </summary>
    public Vector2 localAnchorA;

    /// <summary>
    /// The local anchor point relative to body2's origin.
    /// </summary>
    public Vector2 localAnchorB;

    /// <summary>
    /// The equilibrium length between the anchor points.
    /// </summary>
    public float length;

    /// <summary>
    /// The linear stiffness in N/m. A value of 0 disables softness.
    /// </summary>
    public float stiffness;

    /// <summary>
    /// The linear damping in N*s/m.
    /// </summary>
    public float damping;
  }
}