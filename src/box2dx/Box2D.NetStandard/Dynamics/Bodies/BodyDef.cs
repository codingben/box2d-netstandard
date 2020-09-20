/*
  Box2D.NetStandard Copyright © 2020 Ben Ukhanov & Hugh Phoenix-Hulme https://github.com/benzuk/box2d-netstandard
  Box2DX Copyright (c) 2009 Ihar Kalasouski http://code.google.com/p/box2dx
  
// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
*/

using System.Numerics;

namespace Box2D.NetStandard.Dynamics.Bodies
{
	/// <summary>
	///  A body definition holds all the data needed to construct a rigid body.
	///  You can safely re-use body definitions.
	/// </summary>
	public class BodyDef // in C# it has to be a class to have a parameterless constructor
	{
		/// <summary>
		///  Set this flag to false if this body should never fall asleep. Note that
		///  this increases CPU usage.
		/// </summary>
		public bool allowSleep;

		/// <summary>
		///  The world angle of the body in radians.
		/// </summary>
		public float angle;

		/// <summary>
		///  Angular damping is use to reduce the angular velocity. The damping parameter
		///  can be larger than 1.0f but the damping effect becomes sensitive to the
		///  time step when the damping parameter is large.
		/// </summary>
		public float angularDamping;

		// The angular velocity of the body.
		public float angularVelocity;

		public bool awake;

		/// <summary>
		///  Is this a fast moving body that should be prevented from tunneling through
		///  other moving bodies? Note that all bodies are prevented from tunneling through
		///  static bodies.
		///  @warning You should use this flag sparingly since it increases processing time.
		/// </summary>
		public bool bullet;

		public bool enabled;

		/// <summary>
		///  Should this body be prevented from rotating? Useful for characters.
		/// </summary>
		public bool fixedRotation;

		public float gravityScale;

		/// <summary>
		///  Linear damping is use to reduce the linear velocity. The damping parameter
		///  can be larger than 1.0f but the damping effect becomes sensitive to the
		///  time step when the damping parameter is large.
		/// </summary>
		public float linearDamping;

		/// The linear velocity of the body in world co-ordinates.
		public Vector2 linearVelocity;

		/// <summary>
		///  The world position of the body. Avoid creating bodies at the origin
		///  since this can lead to many overlapping shapes.
		/// </summary>
		public Vector2 position;

		public BodyType type;

		/// <summary>
		///  Use this to store application specific body data.
		/// </summary>
		public object userData;

		/// <summary>
		///  This constructor sets the body definition default values.
		/// </summary>
		public BodyDef()
		{
			userData = null;
			position = Vector2.Zero;
			angle = 0.0f;
			linearVelocity = Vector2.Zero;
			angularVelocity = 0.0f;
			linearDamping = 0.0f;
			angularDamping = 0.0f;
			allowSleep = true;
			awake = true;
			fixedRotation = false;
			bullet = false;
			type = BodyType.Static;
			enabled = true;
			gravityScale = 1.0f;
		}
	}
}