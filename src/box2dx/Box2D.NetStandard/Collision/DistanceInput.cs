using Box2DX.Common;

namespace Box2DX.Collision {
  /// <summary>
  /// Input for Distance.
  /// You have to option to use the shape radii
  /// in the computation.
  /// </summary>
  internal struct DistanceInput {
    internal Transform     transformA;
    internal Transform     transformB;
    internal bool          useRadii;
    internal DistanceProxy proxyA;
    internal DistanceProxy proxyB;
  }
}