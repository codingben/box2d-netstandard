using System.Diagnostics;
using System.Numerics;
using Box2D.NetStandard.Common;

namespace Box2D.NetStandard.Collision {
  /// <summary>
  /// A manifold for two touching convex shapes.
  /// </summary>
  [DebuggerDisplay("localNormal = {" + nameof(localNormal) + "}")]
  public class Manifold {
    /// <summary>
    /// The points of contact.
    /// </summary>
    internal ManifoldPoint[ /*Settings.MaxManifoldPoints*/] points = new ManifoldPoint[Settings.MaxManifoldPoints];

    internal Vector2 localNormal;

    /// <summary>
    /// Usage depends on manifold type.
    /// </summary>
    internal Vector2 localPoint;

    internal ManifoldType type;

    /// <summary>
    /// The number of manifold points.
    /// </summary>
    internal int pointCount;

    internal Manifold() {
      // for (int i = 0; i < Settings.MaxManifoldPoints; i++)
      //   points[i] = new ManifoldPoint();
    }
    
    
  }
}