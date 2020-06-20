using Box2D.NetStandard.Common;

namespace Box2D.NetStandard.Collision {
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