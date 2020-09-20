using Box2D.NetStandard.Dynamics.Fixtures;

namespace Box2D.NetStandard.Dynamics.World.Callbacks
{
	/// <summary>
	///  Implement this class to provide collision filtering. In other words, you can implement
	///  this class if you want finer control over contact creation.
	/// </summary>
	public class ContactFilter
	{
		/// <summary>
		///  Return true if contact calculations should be performed between these two shapes.
		///  If you implement your own collision filter you may want to build from this implementation.
		///  @warning for performance reasons this is only called when the AABBs begin to overlap.
		/// </summary>
		public virtual bool ShouldCollide(Fixture fixtureA, Fixture fixtureB)
		{
			Filter filterA = fixtureA.m_filter;
			Filter filterB = fixtureB.m_filter;

			if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0)
			{
				return filterA.groupIndex > 0;
			}

			bool collide = (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0;
			return collide;
		}

		/// <summary>
		///  Return true if the given shape should be considered for ray intersection.
		/// </summary>
		public bool RayCollide(object userData, Fixture fixture)
		{
			//By default, cast userData as a shape, and then collide if the shapes would collide
			if (userData == null)
			{
				return true;
			}

			return ShouldCollide((Fixture) userData, fixture);
		}
	}
}