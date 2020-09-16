using Box2D.NetStandard.Common;

namespace Box2D.NetStandard.Dynamics.World
{
	/// Contact impulses for reporting. Impulses are used instead of forces because
	/// sub-step forces may approach infinity for rigid body collisions. These
	/// match up one-to-one with the contact points in b2Manifold.
	public class ContactImpulse
	{
		public float[] normalImpulses = new float[Settings.MaxManifoldPoints];
		public float[] tangentImpulses = new float[Settings.MaxManifoldPoints];
	}
}