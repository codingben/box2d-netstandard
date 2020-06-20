using System.Numerics;

namespace Box2D.NetStandard.Collision {
  /// <summary>
  /// Output for Distance.
  /// </summary>
  public struct DistanceOutput {
    /// <summary>
    /// Closest point on shapeA.
    /// </summary>
    public Vector2 pointA;

    /// <summary>
    /// Closest point on shapeB.
    /// </summary>
    public Vector2 pointB;

    public float distance;

    /// <summary>
    /// Number of GJK iterations used.
    /// </summary>
    public int iterations;
  }
}