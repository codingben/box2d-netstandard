using System.Numerics;

namespace Box2DX.Dynamics {
  /// <summary>
  /// A body definition holds all the data needed to construct a rigid body.
  /// You can safely re-use body definitions.
  /// </summary>
  public class BodyDef // in C# it has to be a class to have a parameterless constructor
  {
    /// <summary>
    /// This constructor sets the body definition default values.
    /// </summary>
    public BodyDef()
    {
      userData        = null;
      position        = Vector2.Zero;
      angle           = 0.0f;
      linearVelocity  = Vector2.Zero;
      angularVelocity = 0.0f;
      linearDamping   = 0.0f;
      angularDamping  = 0.0f;
      allowSleep      = true;
      awake           = true;
      fixedRotation   = false;
      bullet          = false;
      type            = BodyType.Static;
      enabled         = true;
      gravityScale    = 1.0f;
    }
		
    /// <summary>
    /// Use this to store application specific body data.
    /// </summary>
    public object userData;

    public BodyType type;
		
    /// <summary>
    /// The world position of the body. Avoid creating bodies at the origin
    /// since this can lead to many overlapping shapes.
    /// </summary>
    public Vector2 position;

    /// <summary>
    /// The world angle of the body in radians.
    /// </summary>
    public float angle;

    /// The linear velocity of the body in world co-ordinates.
    public Vector2 linearVelocity;

    // The angular velocity of the body.
    public float angularVelocity;

    /// <summary>
    /// Linear damping is use to reduce the linear velocity. The damping parameter
    /// can be larger than 1.0f but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    /// </summary>
    public float linearDamping;

    /// <summary>
    /// Angular damping is use to reduce the angular velocity. The damping parameter
    /// can be larger than 1.0f but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    /// </summary>
    public float angularDamping;

    /// <summary>
    /// Set this flag to false if this body should never fall asleep. Note that
    /// this increases CPU usage.
    /// </summary>
    public bool allowSleep;

    /// <summary>
    /// Should this body be prevented from rotating? Useful for characters.
    /// </summary>
    public bool fixedRotation;

    /// <summary>
    /// Is this a fast moving body that should be prevented from tunneling through
    /// other moving bodies? Note that all bodies are prevented from tunneling through
    /// static bodies.
    /// @warning You should use this flag sparingly since it increases processing time.
    /// </summary>
    public bool bullet;

    internal bool  awake;
    public bool  enabled;
    public float gravityScale;
  }
}