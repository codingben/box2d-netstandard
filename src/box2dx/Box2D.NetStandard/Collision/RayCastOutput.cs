using System.Numerics;

namespace Box2DX.Collision {
  /// <summary>
  /// Ray-cast output data.
  /// </summary>
  public struct RayCastOutput {
    public Vector2 normal;
    public float   fraction;
  }
}