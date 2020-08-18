using System;
using Box2D.NetStandard.Collision;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Fixtures;

namespace Box2D.NetStandard.Dynamics.Contacts
{
    internal class ChainAndPolygonContact : Contact
    {
        private static Collider<EdgeShape, PolygonShape> collider = new EdgeAndPolygonCollider();

        private EdgeShape edge;

        public ChainAndPolygonContact(Fixture fA, int indexA, Fixture fB, int indexB) : base(fA, indexA, fB, indexB)
        {
            ChainShape chain = (ChainShape)FixtureA.Shape;
            chain.GetChildEdge(out edge, indexA);
        }

        internal override void Evaluate(out Manifold manifold, in Transform xfA, in Transform xfB)
        {
            collider.Collide(out manifold, edge, xfA, (PolygonShape)FixtureB.Shape, xfB);
        }
    }
}