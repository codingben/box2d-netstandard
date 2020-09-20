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

namespace Box2D.NetStandard.Collision
{
	// Structures and functions used for computing contact points, distance
	// queries, and TOI queries.

	public class Collision
	{
		public static bool TestOverlap(in AABB a, in AABB b)
		{
			Vector2 d1, d2;
			d1 = b.lowerBound - a.upperBound;
			d2 = a.lowerBound - b.upperBound;

			if (d1.X > 0.0f || d1.Y > 0.0f)
			{
				return false;
			}

			if (d2.X > 0.0f || d2.Y > 0.0f)
			{
				return false;
			}

			return true;
		}

		// Sutherland-Hodgman clipping.
		internal static int ClipSegmentToLine(
			out ClipVertex[ /*2*/] vOut,
			in ClipVertex[ /*2*/] vIn,
			in Vector2 normal,
			float offset,
			int vertexIndexA)
		{
			vOut = new ClipVertex[2];

			// Start with no output points
			var numOut = 0;

			// Calculate the distance of end points to the line
			float distance0 = Vector2.Dot(normal, vIn[0].v) - offset;
			float distance1 = Vector2.Dot(normal, vIn[1].v) - offset;

			// If the points are behind the plane
			if (distance0 <= 0.0f)
			{
				vOut[numOut++] = vIn[0];
			}

			if (distance1 <= 0.0f)
			{
				vOut[numOut++] = vIn[1];
			}

			// If the points are on different sides of the plane
			if (distance0 * distance1 < 0.0f)
			{
				// Find intersection point of edge and plane
				float interp = distance0 / (distance0 - distance1);
				vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);

				// VertexA is hitting edgeB.
				vOut[numOut].id.cf.indexA = (byte) vertexIndexA;
				vOut[numOut].id.cf.indexB = vIn[0].id.cf.indexB;
				vOut[numOut].id.cf.typeA = (byte) ContactFeatureType.Vertex;
				vOut[numOut].id.cf.typeB = (byte) ContactFeatureType.Face;
				++numOut;
			}

			return numOut;
		}
	}
}