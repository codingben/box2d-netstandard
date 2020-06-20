using System.Numerics;

namespace Box2DX.Collision {
  internal class SimplexVertex {
    internal Vector2 wA;     // support point in shapeA
    internal Vector2 wB;     // support point in shapeB
    internal Vector2 w;      // wB - wA
    internal float   a;      // barycentric coordinate for closest point
    internal int     indexA; // wA index
    internal int     indexB; // wB index
  }
}