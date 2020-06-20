using Box2D.NetStandard.Collision;
using Box2D.NetStandard.Collision.Shapes;

namespace Box2D.NetStandard.Dynamics.Fixture {
  /// <summary>
  /// A fixture definition is used to create a fixture. This class defines an
  /// abstract fixture definition. You can reuse fixture definitions safely.
  /// </summary>
  public class FixtureDef
  {
    /// <summary>
    /// Use this to store application specific fixture data.
    /// </summary>
    public object userData;

    /// <summary>
    /// The friction coefficient, usually in the range [0,1].
    /// </summary>
    public float friction = 0.2f;

    /// <summary>
    /// The restitution (elasticity) usually in the range [0,1].
    /// </summary>
    public float restitution;

    /// <summary>
    /// The density, usually in kg/m^2.
    /// </summary>
    public float density;

    /// <summary>
    /// A sensor shape collects contact information but never generates a collision response.
    /// </summary>
    public bool isSensor;

    /// <summary>
    /// Contact filtering data.
    /// </summary>
    public Filter Filter = new Filter();

    public Shape shape;
  }

  class FixtureProxy {
    internal AABB aabb;
    internal Fixture fixture;
    internal int childIndex;
    internal int proxyId = -1;
  }
}