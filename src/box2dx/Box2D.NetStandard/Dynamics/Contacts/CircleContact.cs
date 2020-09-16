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
using Box2D.NetStandard.Collision;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Fixtures;

namespace Box2D.NetStandard.Dynamics.Contacts
{
	public class CircleContact : Contact
	{
		private readonly CircleShape circleA;

		private readonly CircleShape circleB;

		public CircleContact(Fixture fA, int indexA, Fixture fB, int indexB) : base(fA, indexA, fB, indexB)
		{
			circleB = (CircleShape) m_fixtureB.Shape;
			circleA = (CircleShape) m_fixtureA.Shape;
		}

		internal override void Evaluate(out Manifold manifold, in Transform xfA, in Transform xfB)
		{
			manifold = new Manifold();
			//manifold.pointCount = 0;

			Vector2 pA = Math.Mul(xfA, circleA.m_p);
			Vector2 pB = Math.Mul(xfB, circleB.m_p);

			Vector2 d = pB - pA;
			float distSqr = Vector2.Dot(d, d);
			float rA = circleA.m_radius, rB = circleB.m_radius;
			float radius = rA + rB;
			if (distSqr > radius * radius)
			{
				return;
			}

			manifold.type = ManifoldType.Circles;
			manifold.localPoint = circleA.m_p;
			manifold.localNormal = Vector2.Zero;
			manifold.pointCount = 1;

			manifold.points[0] = new ManifoldPoint();
			manifold.points[0].localPoint = circleB.m_p;
			manifold.points[0].id.key = 0;
		}
	}
}