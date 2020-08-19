using System.Numerics;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;

namespace Box2D.NetStandard.Collision
{
    internal class CircleAndCircleCollider : Collider<CircleShape, CircleShape>
    {
        internal override void Collide(
          out Manifold manifold,
          in CircleShape circleA,
          in Transform xfA,
          in CircleShape circleB,
          in Transform xfB)
        {
            manifold = new Manifold();
            //manifold.pointCount = 0;

            Vector2 pA = Math.Mul(xfA, circleA.m_p);
            Vector2 pB = Math.Mul(xfB, circleB.m_p);

            Vector2 d = pB - pA;
            float distSqr = Vector2.Dot(d, d);
            float rA = circleA.m_radius, rB = circleB.m_radius;
            float radius = rA + rB;

            if (distSqr > radius * radius)
            {
                return;
            }

            manifold.type = ManifoldType.Circles;
            manifold.localPoint = circleA.m_p;
            manifold.localNormal = Vector2.Zero;
            manifold.pointCount = 1;

            manifold.points[0] = new ManifoldPoint();
            manifold.points[0].localPoint = circleB.m_p;
            manifold.points[0].id.key = 0;
        }
    }
}