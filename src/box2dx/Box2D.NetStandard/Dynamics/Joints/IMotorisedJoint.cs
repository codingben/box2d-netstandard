using System.Runtime.CompilerServices;

namespace Box2D.NetStandard.Dynamics.Joints
{
	public interface IMotorisedJoint
	{
		public float MotorSpeed
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get;
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			set;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void SetMotorSpeed(float speed);

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public float GetMotorSpeed();
	}
}