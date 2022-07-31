using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;

namespace Box2D.NetStandard.Collision
{
    internal abstract class Collider<TShapeA, TShapeB>
        where TShapeA : Shape where TShapeB : Shape
    {
        internal abstract void Collide(
            out Manifold manifold,
            in TShapeA shapeA,
            in Transform xfA,
            in TShapeB shapeB,
            in Transform xfB);
    }
}