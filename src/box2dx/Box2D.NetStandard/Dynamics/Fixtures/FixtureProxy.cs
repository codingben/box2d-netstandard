using Box2D.NetStandard.Collision;

namespace Box2D.NetStandard.Dynamics.Fixtures
{
	internal class FixtureProxy
	{
		internal AABB aabb;
		internal int childIndex;
		internal Fixture fixture;
		internal int proxyId = -1;
	}
}