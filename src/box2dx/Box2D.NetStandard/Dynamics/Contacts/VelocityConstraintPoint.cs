using System.Numerics;

namespace Box2D.NetStandard.Dynamics.Contacts {
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