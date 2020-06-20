/*
  Box2DX Copyright (c) 2009 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;
using Math = Box2D.NetStandard.Common.Math;
using b2Vec2 = System.Numerics.Vector2;

namespace Box2D.NetStandard.Collision.Shapes
{
	/// <summary>
	/// A circle shape.
	/// </summary>
	public class CircleShape : Shape {
		internal Vector2 m_p;

		public CircleShape() {
			m_type = ShapeType.Circle;
			m_radius = 0;
			m_p=Vector2.Zero;
		}

		public Vector2 Center {
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => m_p;
		}
		
		public float Radius {
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => m_radius;
		}

		public override Shape Clone() {
			return (CircleShape) MemberwiseClone();
		}

		public override int GetChildCount() => 1;

		public override bool TestPoint(in Transform transform, in Vector2 p) {
			b2Vec2 center = transform.p + Math.Mul(transform.q, m_p);
			b2Vec2 d      = p           - center;
			return Vector2.Dot(d, d) <= m_radius * m_radius;
		}

		public override bool RayCast(out RayCastOutput output, in RayCastInput input, in Transform transform, int childIndex) {
			output = default;
			
			b2Vec2 position = transform.p + Math.Mul(transform.q, m_p);
			b2Vec2 s        = input.p1    - position;
			float  b        = Vector2.Dot(s, s) - m_radius * m_radius;

			// Solve quadratic equation.
			b2Vec2 r     = input.p2 - input.p1;
			float  c     =  Vector2.Dot(s, r);
			float  rr    =  Vector2.Dot(r,  r);
			float  sigma = c * c - rr * b;

			// Check for negative discriminant and short segment.
			if (sigma < 0.0f || rr < Settings.FLT_EPSILON)
			{
				return false;
			}

			// Find the point of intersection of the line with the circle.
			float a = -(c + MathF.Sqrt(sigma));

			// Is the intersection point on the segment?
			if (0.0f <= a && a <= input.maxFraction * rr)
			{
				a                /= rr;
				output.fraction =  a;
				output.normal   =  Vector2.Normalize(s + a * r);
				return true;
			}

			return false;
		}

		public override void ComputeAABB(out AABB aabb, in Transform transform, int childIndex) {
			b2Vec2 p = transform.p   + Math.Mul(transform.q, m_p);
			aabb.lowerBound=new Vector2(p.X - m_radius, p.Y - m_radius);
			aabb.upperBound=new Vector2(p.X + m_radius, p.Y + m_radius);
		}

		public override void ComputeMass(out MassData massData, float density) {
			massData.mass   = density * Settings.Pi * m_radius * m_radius;
			massData.center = m_p;

			// inertia about the local origin
			massData.I = massData.mass * (0.5f * m_radius * m_radius + Vector2.Dot(m_p, m_p));
		}

		public void Set(in Vector2 center, in float radius) {
			m_p = center;
			m_radius = radius;
		}
	}
}