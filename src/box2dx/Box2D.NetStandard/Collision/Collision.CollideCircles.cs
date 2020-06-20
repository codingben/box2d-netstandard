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

using System.Numerics;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;

namespace Box2D.NetStandard.Collision
{
	public partial class Collision
	{
		internal static void CollideCircles(out Manifold manifold,
			in CircleShape circleA, in Transform xfA, in CircleShape circleB, in Transform xfB)
		{
			manifold=new Manifold();
			manifold.pointCount = 0;

			Vector2 pA = Math.Mul(xfA, circleA.m_p);
			Vector2 pB = Math.Mul(xfB, circleB.m_p);

			Vector2 d = pB - pA;
			float distSqr = Vector2.Dot(d, d);
			float rA = circleA.m_radius, rB= circleB.m_radius;
			float radius = rA + rB;
			if (distSqr > radius*radius)
			{
				return;
			}

			manifold.type = ManifoldType.Circles;
			manifold.localPoint = circleA.m_p;
			manifold.localNormal = Vector2.Zero;
			manifold.pointCount = 1;

			manifold.points[0].localPoint = circleB.m_p;
			manifold.points[0].id.key = 0;
		}

		internal static void CollidePolygonAndCircle(out Manifold manifold,
				in PolygonShape polygonA, in Transform xfA,in CircleShape circleB, Transform xfB)
		{
			manifold=new Manifold();
			
			manifold.pointCount = 0;

			// Compute circle position in the frame of the polygon.
			Vector2 c = Math.Mul(xfB, circleB.m_p);
			Vector2 cLocal = Math.MulT(xfA, c);

			// Find the min separating edge.
			int normalIndex = 0;
			float separation = float.MinValue;
			float radius = polygonA.m_radius + circleB.m_radius;
			int vertexCount = polygonA.m_count;
			Vector2[] vertices = polygonA.m_vertices;
			Vector2[] normals = polygonA.m_normals;

			for (int i = 0; i < vertexCount; ++i)
			{
				float s = Vector2.Dot(normals[i], cLocal - vertices[i]);
				if (s > radius)
				{
					// Early out.
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
				manifold.localNormal = Vector2.Normalize( cLocal - v1);
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