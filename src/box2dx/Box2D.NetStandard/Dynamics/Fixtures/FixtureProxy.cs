using Box2D.NetStandard.Collision;

namespace Box2D.NetStandard.Dynamics.Fixtures
{
    class FixtureProxy
    {
        internal AABB aabb;
        internal Fixture fixture;
        internal int childIndex;
        internal int proxyId = -1;
    }
}