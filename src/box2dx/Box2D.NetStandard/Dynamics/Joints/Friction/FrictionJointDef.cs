using System.Numerics;
using Box2D.NetStandard.Dynamics.Bodies;

namespace Box2D.NetStandard.Dynamics.Joints.Friction
{
	public class FrictionJointDef : JointDef
	{
		public Vector2 localAnchorA;
		public Vector2 localAnchorB;
		public float maxForce;
		public float maxTorque;

		public void Initialize(Body bA, Body bB, in Vector2 anchor)
		{
			bodyA = bA;
			bodyB = bB;
			localAnchorA = bodyA.GetLocalPoint(anchor);
			localAnchorB = bodyB.GetLocalPoint(anchor);
		}
	}
}