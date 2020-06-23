using System.Numerics;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;

namespace Box2D.NetStandard.Collision {
  internal class EdgeAndCircleCollider : Collider<EdgeShape, CircleShape> {
    internal override void Collide(out Manifold manifold, in EdgeShape edgeA, in Transform xfA, in CircleShape circleB, in Transform xfB) {
      manifold = new Manifold();

      //manifold.pointCount = 0;

      Vector2 Q = Math.MulT(xfA, Math.Mul(xfB, circleB.m_p));

      Vector2 A = edgeA.m_vertex1, B = edgeA.m_vertex2;
      Vector2 e = B - A;

      float u = Vector2.Dot(e, B - Q);
      float v = Vector2.Dot(e, Q - A);

      float radius = edgeA.m_radius + circleB.m_radius;

      ContactFeature cf;
      cf.indexB = 0;
      cf.typeB  = (byte) ContactFeatureType.Vertex;
      

      // Region A
      if (v <= 0.0f) {
        Vector2 P  = A;
        Vector2 d  = Q - P;
        float   dd = Vector2.Dot(d, d);
        if (dd > radius * radius) {
          return;
        }

        // Is there an edge connected to A?
        if (edgeA.m_vertex0.HasValue) {
          Vector2 A1 = edgeA.m_vertex0.Value;
          Vector2 B1 = A;
          Vector2 e1 = B1 - A1;
          float   u1 = Vector2.Dot(e1, B1 - Q);

          // Is the circle in Region AB of the previous edge?
          if (u1 > 0.0f) {
            return;
          }
        }

        cf.indexA                     = 0;
        cf.typeA                      = (byte) ContactFeatureType.Vertex;
        manifold.pointCount           = 1;
        manifold.type                 = ManifoldType.Circles;
        manifold.localNormal          = Vector2.Zero;
        manifold.localPoint           = P;
        manifold.points[0]            = new ManifoldPoint();
        manifold.points[0].id.key     = 0;
        manifold.points[0].id.cf      = cf;
        manifold.points[0].localPoint = circleB.m_p;
        return;
      }

      // Region B
      if (u <= 0.0f) {
        Vector2 P  = B;
        Vector2 d  = Q - P;
        float   dd = Vector2.Dot(d, d);
        if (dd > radius * radius) {
          return;
        }

        // Is there an edge connected to B?
        if (edgeA.m_vertex3.HasValue) {
          Vector2 B2 = edgeA.m_vertex3.Value;
          Vector2 A2 = B;
          Vector2 e2 = B2 - A2;
          float   v2 = Vector2.Dot(e2, Q - A2);

          // Is the circle in Region AB of the next edge?
          if (v2 > 0.0f) {
            return;
          }
        }

        cf.indexA                     = 1;
        cf.typeA                      = (byte) ContactFeatureType.Vertex;
        manifold.pointCount           = 1;
        manifold.type                 = ManifoldType.Circles;
        manifold.localNormal          = Vector2.Zero;
        manifold.localPoint           = P;
        manifold.points[0]            = new ManifoldPoint();
        manifold.points[0].id.key     = 0;
        manifold.points[0].id.cf      = cf;
        manifold.points[0].localPoint = circleB.m_p;
        return;
      }

      {
        // Region AB
        float den = Vector2.Dot(e, e);
        //Debug.Assert(den > 0.0f);
        Vector2 P  = (1.0f / den) * (u * A + v * B);
        Vector2 d  = Q - P;
        float   dd = Vector2.Dot(d, d);
        if (dd > radius * radius) {
          return;
        }

        Vector2 n = new Vector2(-e.Y, e.X);
        if (Vector2.Dot(n, Q - A) < 0.0f) {
          n = new Vector2(-n.X, -n.Y);
        }

        n = Vector2.Normalize(n);

        cf.indexA                     = 0;
        cf.typeA                      = (byte) ContactFeatureType.Face;
        manifold.pointCount           = 1;
        manifold.type                 = ManifoldType.FaceA;
        manifold.localNormal          = n;
        manifold.localPoint           = A;
        manifold.points[0]            = new ManifoldPoint();
        manifold.points[0].id.key     = 0;
        manifold.points[0].id.cf      = cf;
        manifold.points[0].localPoint = circleB.m_p;
      }
    }
  }
}