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
	/// <summary>
	///  A manifold point is a contact point belonging to a contact
	///  manifold. It holds details related to the geometry and dynamics
	///  of the contact points.
	///  The local point usage depends on the manifold type:
	///  -Circles: the local center of circleB
	///  -FaceA: the local center of cirlceB or the clip point of polygonB
	///  -FaceB: the clip point of polygonA
	///  This structure is stored across time steps, so we keep it small.
	///  Note: the impulses are used for internal caching and may not
	///  provide reliable contact forces, especially for high speed collisions.
	/// </summary>
	internal class ManifoldPoint
	{
		/// <summary>
		///  Uniquely identifies a contact point between two shapes.
		/// </summary>
		internal ContactID id;

		/// <summary>
		///  Usage depends on manifold type.
		/// </summary>
		internal Vector2 localPoint;

		/// <summary>
		///  The non-penetration impulse.
		/// </summary>
		internal float normalImpulse;

		/// <summary>
		///  The friction impulse.
		/// </summary>
		internal float tangentImpulse;
	}
}