using System.Numerics;
using Box2DX.Common;
using int32 = System.Int32;
using b2Vec2 = System.Numerics.Vector2;

namespace Box2DX.Collision {
  /// <summary>
  /// This is used to compute the current state of a contact manifold.
  /// </summary>
  internal class WorldManifold {
    /// <summary>
    /// World vector pointing from A to B.
    /// </summary>
    internal Vector2 normal;

    /// <summary>
    /// World contact point (point of intersection).
    /// </summary>
    internal Vector2[] points = new Vector2[Settings.MaxManifoldPoints];
    float[] separations = new float[Settings.MaxManifoldPoints];

    /// Evaluate the manifold with supplied transforms. This assumes
    /// modest motion from the original state. This does not change the
    /// point count, impulses, etc. The radii must come from the shapes
    /// that generated the manifold.
    internal void Initialize(Manifold manifold, Transform xfA, float radiusA, Transform xfB, float radiusB) {
      if (manifold.pointCount == 0) {
        return;
      }

      switch (manifold.type) {
        case ManifoldType.Circles: {
          normal = new Vector2(1.0f, 0.0f);
          Vector2 pointA = Common.Math.Mul(xfA, manifold.localPoint);
          Vector2 pointB = Common.Math.Mul(xfB, manifold.points[0].localPoint);
          if (Vector2.DistanceSquared(pointA, pointB) > Settings.FLT_EPSILON_SQUARED) {
            normal = Vector2.Normalize(pointB - pointA);
          }

          Vector2 cA = pointA    + radiusA * normal;
          Vector2 cB = pointB    - radiusB * normal;
          points[0] = 0.5f * (cA + cB);
          separations[0] = Vector2.Dot(cB - cA, normal);
        }
          break;

        case ManifoldType.FaceA: {
          normal     = Math.Mul(xfA.q, manifold.localNormal);
          Vector2 planePoint = Math.Mul(xfA,   manifold.localPoint);

          for (int i = 0; i < manifold.pointCount; ++i) {
            Vector2 clipPoint = Math.Mul(xfB, manifold.points[i].localPoint);
            Vector2 cA        = clipPoint + (radiusA - Vector2.Dot(clipPoint - planePoint, normal)) * normal;
            Vector2 cB        = clipPoint - radiusB                                                 * normal;
            points[i] = 0.5f * (cA        + cB);
            separations[i] = Vector2.Dot(cB - cA, normal);
          }
        }
          break;

        case ManifoldType.FaceB: {
          normal     = Math.Mul(xfB.q, manifold.localNormal);
          Vector2 planePoint = Math.Mul(xfB,   manifold.localPoint);
          
          for (int i = 0; i < manifold.pointCount; ++i) {
            Vector2 clipPoint = Math.Mul(xfA, manifold.points[i].localPoint);
            Vector2 cB = clipPoint + (radiusB - Vector2.Dot(clipPoint - planePoint, normal)) * normal;
            Vector2 cA        = clipPoint - radiusA                                                 * normal;
            
            points[i] = 0.5f * (cA        + cB);
            separations[i] = Vector2.Dot(cA - cB, normal);
          }

          // Ensure normal points from A to B.
          normal = -normal;
        }
          break;
      }
    }

   
  }
}