using System;

namespace Box2D.NetStandard.Dynamics.World.Callbacks
{
	[Flags]
	public enum DrawFlags
	{
		Shape = 0x0001,       // draw shapes
		Joint = 0x0002,       // draw joint connections
		Aabb = 0x0008,        // draw axis aligned bounding boxes
		Pair = 0x0020,        // draw broad-phase pairs
		CenterOfMass = 0x0040 // draw center of mass frame
	}
}