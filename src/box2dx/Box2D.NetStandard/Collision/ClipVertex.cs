using System.Numerics;

namespace Box2DX.Collision {
  /// <summary>
  /// Used for computing contact manifolds.
  /// </summary>
  internal struct ClipVertex {
    internal Vector2   v;
    internal ContactID id;
  }
}