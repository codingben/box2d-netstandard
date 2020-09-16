using System.Numerics;
using Box2D.NetStandard.Dynamics.Bodies;

namespace Box2D.NetStandard.Dynamics.Joints.Motor
{
	public class MotorJointDef : JointDef
	{
		/// The bodyB angle minus bodyA angle in radians.
		public float angularOffset;

		/// Position correction factor in the range [0,1].
		public float correctionFactor;

		/// Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
		public Vector2 linearOffset;

		/// The maximum motor force in N.
		public float maxForce;

		/// The maximum motor torque in N-m.
		public float maxTorque;

		private MotorJointDef()
		{
			maxForce = 1.0f;
			maxTorque = 1.0f;
			correctionFactor = 0.3f;
		}

		/// Initialize the bodies and offsets using the current transforms.
		public void Initialize(Body bA, Body bB)
		{
			bodyA = bA;
			bodyB = bB;
			Vector2 xB = bodyB.GetPosition();
			linearOffset = bodyA.GetLocalPoint(xB);

			float angleA = bodyA.GetAngle();
			float angleB = bodyB.GetAngle();
			angularOffset = angleB - angleA;
		}
	}
}