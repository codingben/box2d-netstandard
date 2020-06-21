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


using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Joints.Distance;
using Box2D.NetStandard.Dynamics.Joints.Gear;
using Box2D.NetStandard.Dynamics.Joints.Mouse;
using Box2D.NetStandard.Dynamics.Joints.Prismatic;
using Box2D.NetStandard.Dynamics.Joints.Pulley;
using Box2D.NetStandard.Dynamics.Joints.Revolute;
using Box2D.NetStandard.Dynamics.Joints.Wheel;
using Box2D.NetStandard.Dynamics.World;

namespace Box2D.NetStandard.Dynamics.Joints
{
	/// <summary>
	/// The base joint class. Joints are used to constraint two bodies together in
	/// various fashions. Some joints also feature limits and motors.
	/// </summary>
	public abstract class Joint
	{
		protected JointType _type;
		internal Joint _prev;
		internal Joint _next;
		internal JointEdge _edgeA = new JointEdge();
		internal JointEdge _edgeB = new JointEdge();
		internal Body _bodyA;
		internal Body _bodyB;

		internal bool _islandFlag;
		internal bool _collideConnected;

		protected object _userData;

		// Cache here per time step to reduce cache misses.
		protected Vector2 _localCenter1, _localCenter2;
		protected float _invMass1, _invI1;
		protected float _invMass2, _invI2;

		/// <summary>
		/// Get the type of the concrete joint.
		/// </summary>
		public JointType Type => _type;
		
		/// <summary>
		/// Get the first body attached to this joint.
		/// </summary>
		/// <returns></returns>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public Body GetBodyA() => _bodyA;

		/// <summary>
		/// Get the second body attached to this joint.
		/// </summary>
		/// <returns></returns>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public Body GetBodyB() => _bodyB;

		/// <summary>
		/// Get the anchor point on body1 in world coordinates.
		/// </summary>
		/// <returns></returns>
		public abstract Vector2 GetAnchorA { get; }

		/// <summary>
		/// Get the anchor point on body2 in world coordinates.
		/// </summary>
		/// <returns></returns>
		public abstract Vector2 GetAnchorB { get; }

		/// <summary>
		/// Get the reaction force on body2 at the joint anchor.
		/// </summary>		
		public abstract Vector2 GetReactionForce(float inv_dt);

		/// <summary>
		/// Get the reaction torque on body2.
		/// </summary>		
		public abstract float GetReactionTorque(float inv_dt);

		/// <summary>
		/// Get the next joint the world joint list.
		/// </summary>
		/// <returns></returns>
		[MethodImpl(MethodImplOptions.AggressiveInlining)] 
		public Joint GetNext() => _next;

		/// <summary>
		/// Get/Set the user data pointer.
		/// </summary>
		/// <returns></returns>
		public object UserData
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => _userData;
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			set => _userData = value;
		}

		protected Joint(JointDef def)
		{
			_type = def.Type;
			_prev = null;
			_next = null;
			_bodyA = def.BodyA;
			_bodyB = def.BodyB;
			_collideConnected = def.CollideConnected;
			_islandFlag = false;
			_userData = def.UserData;
		}

		internal static Joint Create(JointDef def)
		{
			Joint joint = null;

			switch (def.Type) {
				case JointType.DistanceJoint:
					joint = new DistanceJoint((DistanceJointDef) def);
					break;
				case JointType.MouseJoint:
					joint = new MouseJoint((MouseJointDef) def);
					break;
				case JointType.PrismaticJoint:
					joint = new PrismaticJoint((PrismaticJointDef) def);
					break;
				case JointType.RevoluteJoint:
					joint = new RevoluteJoint((RevoluteJointDef) def);
					break;
				case JointType.PulleyJoint:
					joint = new PulleyJoint((PulleyJointDef) def);
					break;
				case JointType.GearJoint:
					joint = new GearJoint((GearJointDef) def);
					break;
				case JointType.WheelJoint:
					joint = new WheelJoint((WheelJointDef) def);
					break;

				default:
					Debug.Assert(false);
					break;
			}

			return joint;
		}

		internal abstract void InitVelocityConstraints(in SolverData data);
		internal abstract void SolveVelocityConstraints(in SolverData data);

		// This returns true if the position errors are within tolerance.
		internal abstract bool SolvePositionConstraints(in SolverData data);

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal void ComputeXForm(ref Transform xf, Vector2 center, Vector2 localCenter, float angle)
		{
			xf.q = Matrix3x2.CreateRotation(angle); // .Set(angle);
			xf.p = center - Math.Mul(xf.q, localCenter);
		}
	}
}
