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
using Box2D.NetStandard.Common;

namespace Box2D.NetStandard.Collision
{
	/// <summary>
	///  This is used to compute the current state of a contact manifold.
	/// </summary>
	internal class WorldManifold
	{
		private readonly float[] separations = new float[Settings.MaxManifoldPoints];

		/// <summary>
		///  World vector pointing from A to B.
		/// </summary>
		internal Vector2 normal;

		/// <summary>
		///  World contact point (point of intersection).
		/// </summary>
		internal Vector2[] points = new Vector2[Settings.MaxManifoldPoints];

		/// Evaluate the manifold with supplied transforms. This assumes
		/// modest motion from the original state. This does not change the
		/// point count, impulses, etc. The radii must come from the shapes
		/// that generated the manifold.
		internal void Initialize(
			Manifold manifold,
			Transform xfA,
			float radiusA,
			Transform xfB,
			float radiusB)
		{
			if (manifold.pointCount == 0)
			{
				return;
			}

			switch (manifold.type)
			{
				case ManifoldType.Circles: {
					normal = new Vector2(1.0f, 0.0f);
					Vector2 pointA = Math.Mul(xfA, manifold.localPoint);
					Vector2 pointB = Math.Mul(xfB, manifold.points[0].localPoint);
					if (Vector2.DistanceSquared(pointA, pointB) > Settings.FLT_EPSILON_SQUARED)
					{
						normal = Vector2.Normalize(pointB - pointA);
					}

					Vector2 cA = pointA + radiusA * normal;
					Vector2 cB = pointB - radiusB * normal;
					points[0] = 0.5f * (cA + cB);
					separations[0] = Vector2.Dot(cB - cA, normal);
				}
					break;

				case ManifoldType.FaceA: {
					normal = Vector2.Transform(manifold.localNormal, xfA.q); // Math.Mul(xfA.q, manifold.localNormal);
					Vector2 planePoint = Math.Mul(xfA, manifold.localPoint);

					for (var i = 0; i < manifold.pointCount; ++i)
					{
						Vector2 clipPoint = Math.Mul(xfB, manifold.points[i].localPoint);
						Vector2 cA = clipPoint + (radiusA - Vector2.Dot(clipPoint - planePoint, normal)) * normal;
						Vector2 cB = clipPoint - radiusB * normal;
						points[i] = 0.5f * (cA + cB);
						separations[i] = Vector2.Dot(cB - cA, normal);
					}
				}
					break;

				case ManifoldType.FaceB: {
					normal = Vector2.Transform(manifold.localNormal, xfB.q); // Math.Mul(xfB.q, manifold.localNormal);
					Vector2 planePoint = Math.Mul(xfB, manifold.localPoint);

					for (var i = 0; i < manifold.pointCount; ++i)
					{
						Vector2 clipPoint = Math.Mul(xfA, manifold.points[i].localPoint);
						Vector2 cB = clipPoint + (radiusB - Vector2.Dot(clipPoint - planePoint, normal)) * normal;
						Vector2 cA = clipPoint - radiusA * normal;

						points[i] = 0.5f * (cA + cB);
						separations[i] = Vector2.Dot(cA - cB, normal);
					}

					// Ensure normal points from A to B.
					normal = -normal;
				}
					break;
			}
		}
	}
}