using System.Numerics;
using Box2D.NetStandard.Dynamics.Bodies;

namespace Box2D.NetStandard.Dynamics.Joints.Pulley
{
	/// <summary>
	///  Pulley joint definition. This requires two ground anchors,
	///  two dynamic body anchor points, max lengths for each side,
	///  and a pulley ratio.
	/// </summary>
	public class PulleyJointDef : JointDef
	{
		/// <summary>
		///  The first ground anchor in world coordinates. This point never moves.
		/// </summary>
		public Vector2 GroundAnchorA;

		/// <summary>
		///  The second ground anchor in world coordinates. This point never moves.
		/// </summary>
		public Vector2 GroundAnchorB;

		/// <summary>
		///  The a reference length for the segment attached to body1.
		/// </summary>
		public float LengthA;

		/// <summary>
		///  The a reference length for the segment attached to body2.
		/// </summary>
		public float LengthB;

		/// <summary>
		///  The local anchor point relative to body1's origin.
		/// </summary>
		public Vector2 LocalAnchorA;

		/// <summary>
		///  The local anchor point relative to body2's origin.
		/// </summary>
		public Vector2 LocalAnchorB;

		/// <summary>
		///  The maximum length of the segment attached to body1.
		/// </summary>
		public float MaxLength1;

		/// <summary>
		///  The maximum length of the segment attached to body2.
		/// </summary>
		public float MaxLength2;

		/// <summary>
		///  The pulley ratio, used to simulate a block-and-tackle.
		/// </summary>
		public float Ratio;

		public PulleyJointDef()
		{
			GroundAnchorA = new Vector2(-1.0f, 1.0f);
			GroundAnchorB = new Vector2(1.0f, 1.0f);
			LocalAnchorA = new Vector2(-1.0f, 0.0f);
			LocalAnchorB = new Vector2(1.0f, 0.0f);
			Ratio = 1.0f;
			collideConnected = true;
		}

		/// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
		public void Initialize(
			Body body1,
			Body body2,
			Vector2 groundAnchor1,
			Vector2 groundAnchor2,
			Vector2 anchor1,
			Vector2 anchor2,
			float ratio)
		{
			bodyA = body1;
			bodyB = body2;
			GroundAnchorA = groundAnchor1;
			GroundAnchorB = groundAnchor2;
			LocalAnchorA = body1.GetLocalPoint(anchor1);
			LocalAnchorB = body2.GetLocalPoint(anchor2);
			Vector2 dA = anchor1 - groundAnchor1;
			LengthA = dA.Length();
			Vector2 dB = anchor2 - groundAnchor2;
			LengthB = dB.Length();
			Ratio = ratio;
			//Debug.Assert(ratio > Settings.FLT_EPSILON);
		}
	}
}