using System.Numerics;

namespace Box2DX.Collision {
  /// <summary>
  /// Ray-cast input data.
  /// </summary>
  public struct RayCastInput {
    public Vector2 p1, p2;
    public float   maxFraction;
  }
}