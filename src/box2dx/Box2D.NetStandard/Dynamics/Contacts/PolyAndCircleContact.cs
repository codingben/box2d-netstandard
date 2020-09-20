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
	internal class PolyAndCircleContact : Contact
	{
		private readonly CircleShape circleB;
		private readonly PolygonShape polygonA;

		public PolyAndCircleContact(Fixture fA, int indexA, Fixture fB, int indexB) : base(fA, indexA, fB, indexB)
		{
			polygonA = (PolygonShape) m_fixtureA.Shape;
			circleB = (CircleShape) m_fixtureB.Shape;
		}

		internal override void Evaluate(out Manifold manifold, in Transform xfA, in Transform xfB)
		{
			manifold = new Manifold();

			//manifold.pointCount = 0;

			// Compute circle position in the frame of the polygon.
			Vector2 c = Math.Mul(xfB, circleB.m_p);
			Vector2 cLocal = Math.MulT(xfA, c);

			// Find the min separating edge.
			var normalIndex = 0;
			float separation = float.MinValue;
			float radius = polygonA.m_radius + circleB.m_radius;
			int vertexCount = polygonA.m_count;
			Vector2[] vertices = polygonA.m_vertices;
			Vector2[] normals = polygonA.m_normals;

			for (var i = 0; i < vertexCount; ++i)
			{
				float s = Vector2.Dot(normals[i], cLocal - vertices[i]);
				if (s > radius)
					// Early out.
				{
					return;
				}

				if (s > separation)
				{
					separation = s;
					normalIndex = i;
				}
			}

			// Vertices that subtend the incident face.
			int vertIndex1 = normalIndex;
			int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
			Vector2 v1 = vertices[vertIndex1];
			Vector2 v2 = vertices[vertIndex2];
			manifold.points[0] = new ManifoldPoint();

			// If the center is inside the polygon ...
			if (separation < Settings.FLT_EPSILON)
			{
				manifold.pointCount = 1;
				manifold.type = ManifoldType.FaceA;
				manifold.localNormal = normals[normalIndex];
				manifold.localPoint = 0.5f * (v1 + v2);
				manifold.points[0].localPoint = circleB.m_p;
				manifold.points[0].id.key = 0;
				return;
			}

			// Compute barycentric coordinates
			float u1 = Vector2.Dot(cLocal - v1, v2 - v1);
			float u2 = Vector2.Dot(cLocal - v2, v1 - v2);
			if (u1 <= 0.0f)
			{
				if (Vector2.DistanceSquared(cLocal, v1) > radius * radius)
				{
					return;
				}

				manifold.pointCount = 1;
				manifold.type = ManifoldType.FaceA;
				manifold.localNormal = Vector2.Normalize(cLocal - v1);
				manifold.localPoint = v1;
				manifold.points[0].localPoint = circleB.m_p;
				manifold.points[0].id.key = 0;
			}
			else if (u2 <= 0.0f)
			{
				if (Vector2.DistanceSquared(cLocal, v2) > radius * radius)
				{
					return;
				}

				manifold.pointCount = 1;
				manifold.type = ManifoldType.FaceA;
				manifold.localNormal = Vector2.Normalize(cLocal - v2);
				manifold.localPoint = v2;
				manifold.points[0].localPoint = circleB.m_p;
				manifold.points[0].id.key = 0;
			}
			else
			{
				Vector2 faceCenter = 0.5f * (v1 + v2);
				float s = Vector2.Dot(cLocal - faceCenter, normals[vertIndex1]);
				if (s > radius)
				{
					return;
				}

				manifold.pointCount = 1;
				manifold.type = ManifoldType.FaceA;
				manifold.localNormal = normals[vertIndex1];
				manifold.localPoint = faceCenter;
				manifold.points[0].localPoint = circleB.m_p;
				manifold.points[0].id.key = 0;
			}
		}
	}
}