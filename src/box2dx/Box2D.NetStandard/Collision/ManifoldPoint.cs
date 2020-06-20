using System.Numerics;

namespace Box2DX.Collision {
  /// <summary>
  /// A manifold point is a contact point belonging to a contact
  /// manifold. It holds details related to the geometry and dynamics
  /// of the contact points.
  /// The local point usage depends on the manifold type:
  /// -Circles: the local center of circleB
  /// -FaceA: the local center of cirlceB or the clip point of polygonB
  /// -FaceB: the clip point of polygonA
  /// This structure is stored across time steps, so we keep it small.
  /// Note: the impulses are used for internal caching and may not
  /// provide reliable contact forces, especially for high speed collisions.
  /// </summary>
  internal class ManifoldPoint {
    /// <summary>
    /// Usage depends on manifold type.
    /// </summary>
    internal Vector2 localPoint;

    /// <summary>
    /// The non-penetration impulse.
    /// </summary>
    internal float normalImpulse;

    /// <summary>
    /// The friction impulse.
    /// </summary>
    internal float tangentImpulse;

    /// <summary>
    /// Uniquely identifies a contact point between two shapes.
    /// </summary>
    internal ContactID id;
    
  }
}