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

using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Joints.Distance;
using Box2D.NetStandard.Dynamics.Joints.Friction;
using Box2D.NetStandard.Dynamics.Joints.Gear;
using Box2D.NetStandard.Dynamics.Joints.Mouse;
using Box2D.NetStandard.Dynamics.Joints.Prismatic;
using Box2D.NetStandard.Dynamics.Joints.Pulley;
using Box2D.NetStandard.Dynamics.Joints.Revolute;
using Box2D.NetStandard.Dynamics.Joints.Weld;
using Box2D.NetStandard.Dynamics.Joints.Wheel;
using Box2D.NetStandard.Dynamics.World;
using Box2D.NetStandard.Dynamics.World.Callbacks;

namespace Box2D.NetStandard.Dynamics.Joints
{
	/// <summary>
	///  The base joint class. Joints are used to constraint two bodies together in
	///  various fashions. Some joints also feature limits and motors.
	/// </summary>
	public abstract class Joint
	{
		internal readonly bool m_collideConnected;
		internal readonly JointEdge m_edgeA = new JointEdge();
		internal readonly JointEdge m_edgeB = new JointEdge();
		internal Body m_bodyA;
		internal Body m_bodyB;
		protected float m_invMass1, m_invI1;
		protected float m_invMass2, m_invI2;

		internal bool m_islandFlag;

		// Cache here per time step to reduce cache misses.
		protected Vector2 m_localCenter1, m_localCenter2;
		internal Joint m_next;
		internal Joint m_prev;

		protected Joint(JointDef def)
		{
			m_prev = null;
			m_next = null;
			m_bodyA = def.bodyA;
			m_bodyB = def.bodyB;
			m_collideConnected = def.collideConnected;
			m_islandFlag = false;
			UserData = def.UserData;
		}

		/// <summary>
		///  Get the anchor point on body1 in world coordinates.
		/// </summary>
		/// <returns></returns>
		public abstract Vector2 GetAnchorA { get; }

		/// <summary>
		///  Get the anchor point on body2 in world coordinates.
		/// </summary>
		/// <returns></returns>
		public abstract Vector2 GetAnchorB { get; }

		/// <summary>
		///  Get/Set the user data pointer.
		/// </summary>
		/// <returns></returns>
		public object UserData
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get;
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			set;
		}

		public static void LinearStiffness(
			out float stiffness,
			out float damping,
			in float frequencyHz,
			in float dampingRatio,
			in Body bodyA,
			in Body bodyB)
		{
			float massA = bodyA.GetMass();
			float massB = bodyB.GetMass();
			float mass;

			if (massA > 0.0f && massB > 0.0f)
			{
				mass = massA * massB / (massA + massB);
			}
			else if (massA > 0.0f)
			{
				mass = massA;
			}
			else
			{
				mass = massB;
			}

			float omega = 2.0f * Settings.Pi * frequencyHz;
			stiffness = mass * omega * omega;
			damping = 2.0f * mass * dampingRatio * omega;
		}

		public static void AngularStiffness(
			out float stiffness,
			out float damping,
			in float frequencyHz,
			in float dampingRatio,
			in Body bodyA,
			in Body bodyB)
		{
			float IA = bodyA.GetInertia();
			float IB = bodyB.GetInertia();
			float I;

			if (IA > 0.0f && IB > 0.0f)
			{
				I = IA * IB / (IA + IB);
			}
			else if (IA > 0.0f)
			{
				I = IA;
			}
			else
			{
				I = IB;
			}

			float omega = 2.0f * Settings.Pi * frequencyHz;
			stiffness = I * omega * omega;
			damping = 2.0f * I * dampingRatio * omega;
		}

		/// <summary>
		///  Get the first body attached to this joint.
		/// </summary>
		/// <returns></returns>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public Body GetBodyA() => m_bodyA;

		/// <summary>
		///  Get the second body attached to this joint.
		/// </summary>
		/// <returns></returns>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public Body GetBodyB() => m_bodyB;

		/// <summary>
		///  Get the reaction force on body2 at the joint anchor.
		/// </summary>
		public abstract Vector2 GetReactionForce(float inv_dt);

		/// <summary>
		///  Get the reaction torque on body2.
		/// </summary>
		public abstract float GetReactionTorque(float inv_dt);

		/// <summary>
		///  Get the next joint the world joint list.
		/// </summary>
		/// <returns></returns>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public Joint GetNext() => m_next;

		internal static Joint Create(JointDef def)
		{
			return def switch
            {
                DistanceJointDef d  => new DistanceJoint(d),
                MouseJointDef d     => new MouseJoint(d),
                PrismaticJointDef d => new PrismaticJoint(d),
                RevoluteJointDef d  => new RevoluteJoint(d),
                PulleyJointDef d    => new PulleyJoint(d),
                GearJointDef d      => new GearJoint(d),
                WheelJointDef d     => new WheelJoint(d),
                WeldJointDef d      => new WeldJoint(d),
				FrictionJointDef d  => new FrictionJoint(d),
                _ => throw new NotImplementedException($"JointDef '{def.GetType().Name}' is not implemented.")
            };
		}

		internal abstract void InitVelocityConstraints(in SolverData data);
		internal abstract void SolveVelocityConstraints(in SolverData data);

		// This returns true if the position errors are within tolerance.
		internal abstract bool SolvePositionConstraints(in SolverData data);

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal void ComputeXForm(ref Transform xf, Vector2 center, Vector2 localCenter, float angle)
		{
			xf.q = Matrex.CreateRotation(angle);                  // Actually about twice as fast to use our own function
			xf.p = center - Vector2.Transform(localCenter, xf.q); // Math.Mul(xf.q, localCenter);
		}

		public void Draw(DebugDraw draw)
		{
			Transform xf1 = m_bodyA.GetTransform();
			Transform xf2 = m_bodyB.GetTransform();
			Vector2 x1 = xf1.p;
			Vector2 x2 = xf2.p;
			Vector2 p1 = GetAnchorA;
			Vector2 p2 = GetAnchorB;

			var color = new Color(0.5f, 0.8f, 0.8f);

			switch (this)
			{
				case DistanceJoint j:
					draw.DrawSegment(p1, p2, color);
					break;
				case PulleyJoint pulley: {
					Vector2 s1 = pulley.GroundAnchorA;
					Vector2 s2 = pulley.GroundAnchorB;
					draw.DrawSegment(s1, p1, color);
					draw.DrawSegment(s2, p2, color);
					draw.DrawSegment(s1, s2, color);
				}
					break;

				case MouseJoint j: {
					var c = new Color();
					c.Set(0.0f, 1.0f, 0.0f);
					draw.DrawPoint(p1, 4.0f, c);
					draw.DrawPoint(p2, 4.0f, c);

					c.Set(0.8f, 0.8f, 0.8f);
					draw.DrawSegment(p1, p2, c);
				}
					break;

				default:
					draw.DrawSegment(x1, p1, color);
					draw.DrawSegment(p1, p2, color);
					draw.DrawSegment(x2, p2, color);
					break;
			}
		}
	}
}