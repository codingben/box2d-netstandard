namespace Box2D.NetStandard.Collision
{
    /// The features that intersect to form the contact point
    /// This must be 4 bytes or less.
    internal struct ContactFeature
    {
        internal byte indexA;

        ///< Feature index on shapeA
        internal byte indexB;

        ///< Feature index on shapeB
        internal byte typeA;

        ///< The feature type on shapeA
        internal byte typeB; ///< The feature type on shapeB
	}
}