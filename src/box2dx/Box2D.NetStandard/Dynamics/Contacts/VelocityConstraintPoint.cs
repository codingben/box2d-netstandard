using System.Numerics;

namespace Box2DX.Dynamics {
  internal class VelocityConstraintPoint {
    internal Vector2 rA;
    internal Vector2 rB;
    internal float   normalImpulse;
    internal float   tangentImpulse;
    internal float   normalMass;
    internal float   tangentMass;
    internal float   velocityBias;
  };
}