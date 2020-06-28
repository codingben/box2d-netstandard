using Box2D.NetStandard.Collision;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Fixtures;

namespace Box2D.NetStandard.Dynamics.Contacts {
  internal class ChainAndCircleContact : Contact {
    
    private static Collider<EdgeShape, CircleShape> collider = new EdgeAndCircleCollider();

    private EdgeShape edge;
    
    public ChainAndCircleContact(Fixture fA, int indexA, Fixture fB, int indexB) : base(fA, indexA, fB, indexB) {
      ChainShape chain = (ChainShape)FixtureA.Shape;
      chain.GetChildEdge(out edge, indexA);
    }
    internal override void Evaluate(out Manifold manifold, in Transform xfA, in Transform xfB) {
      collider.Collide(out manifold, edge, xfA, (CircleShape)FixtureB.Shape, xfB );
    }
  }
}