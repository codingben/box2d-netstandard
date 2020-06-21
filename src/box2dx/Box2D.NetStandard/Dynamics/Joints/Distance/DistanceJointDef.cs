using System.Numerics;
using Box2D.NetStandard.Dynamics.Bodies;

namespace Box2D.NetStandard.Dynamics.Joints.Distance {
  /// <summary>
  /// Distance joint definition. This requires defining an
  /// anchor point on both bodies and the non-zero length of the
  /// distance joint. The definition uses local anchor points
  /// so that the initial configuration can violate the constraint
  /// slightly. This helps when saving and loading a game.
  /// @warning Do not use a zero or short length.
  /// </summary>
  public class DistanceJointDef : JointDef {
    public DistanceJointDef() {
      Type         = JointType.DistanceJoint;
      LocalAnchorA = new Vector2(0.0f, 0.0f);
      LocalAnchorB = new Vector2(0.0f, 0.0f);
      Length       = 1.0f;
      FrequencyHz  = 0.0f;
      DampingRatio = 0.0f;
    }

    /// <summary>
    /// Initialize the bodies, anchors, and length using the world anchors.
    /// </summary>
    public void Initialize(Body body1, Body body2, Vector2 anchor1, Vector2 anchor2) {
      BodyA        = body1;
      BodyB        = body2;
      LocalAnchorA = body1.GetLocalPoint(anchor1);
      LocalAnchorB = body2.GetLocalPoint(anchor2);
      Vector2 d = anchor2 - anchor1;
      Length = d.Length();
    }

    /// <summary>
    /// The local anchor point relative to body1's origin.
    /// </summary>
    public Vector2 LocalAnchorA;

    /// <summary>
    /// The local anchor point relative to body2's origin.
    /// </summary>
    public Vector2 LocalAnchorB;

    /// <summary>
    /// The equilibrium length between the anchor points.
    /// </summary>
    public float Length;

    /// <summary>
    /// The response speed.
    /// </summary>
    public float FrequencyHz;

    /// <summary>
    /// The damping ratio. 0 = no damping, 1 = critical damping.
    /// </summary>
    public float DampingRatio;
  }
}