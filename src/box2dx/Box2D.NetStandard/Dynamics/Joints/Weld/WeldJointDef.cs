using System;
using System.Numerics;

namespace Box2D.NetStandard.Dynamics.Joints.Weld
{
	public class WeldJointDef : JointDef
	{
		/// <summary>
		///  The rotational damping in N*m*s
		/// </summary>
		public float damping;

		[Obsolete("Use Joint.AngularStiffness to get stiffness & damping values", true)]
		public float dampingRatio;

		[Obsolete("Use Joint.AngularStiffness to get stiffness & damping values", true)]
		public float frequencyHz;

		public Vector2 localAnchorA;
		public Vector2 localAnchorB;
		public float referenceAngle;

		/// <summary>
		///  The rotational stiffness in N*m
		///  Disable softness with a value of 0
		/// </summary>
		public float stiffness;
	}
}