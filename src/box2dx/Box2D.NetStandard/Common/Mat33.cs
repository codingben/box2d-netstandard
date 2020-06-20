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

namespace Box2D.NetStandard.Common
{
	/// <summary>
	/// A 3-by-3 matrix. Stored in column-major order.
	/// </summary>
	public struct Mat33
	{
		/// <summary>
		/// Construct this matrix using columns.
		/// </summary>
		public Mat33(Vector3 c1, Vector3 c2, Vector3 c3)
		{
			ex = c1;
			ey = c2;
			ez = c3;
		}

		/// <summary>
		/// Set this matrix to all zeros.
		/// </summary>
		public void SetZero()
		{
			ex = Vector3.Zero;
			ey = Vector3.Zero;
			ez = Vector3.Zero;
		}

		/// <summary>
		/// Solve A * x = b, where b is a column vector. This is more efficient
		/// than computing the inverse in one-shot cases.
		/// </summary>
		public Vector3 Solve33(Vector3 b)
		{
			float det = Vector3.Dot(ex, Vector3.Cross(ey, ez));
			Debug.Assert(det != 0.0f);
			det = 1.0f / det;
			Vector3 x = new Vector3();
			x.X = det * Vector3.Dot(b,  Vector3.Cross(ey, ez));
			x.Y = det * Vector3.Dot(ex, Vector3.Cross(b, ez));
			x.Z = det * Vector3.Dot(ex, Vector3.Cross(ey, b));
			return x;
		}

		/// <summary>
		/// Solve A * x = b, where b is a column vector. This is more efficient
		/// than computing the inverse in one-shot cases. Solve only the upper
		/// 2-by-2 matrix equation.
		/// </summary>
		public Vector2 Solve22(Vector2 b)
		{
			float a11 = ex.X, a12 = ey.X, a21 = ex.Y, a22 = ey.Y;
			float det = a11 * a22 - a12 * a21;
			Debug.Assert(det != 0.0f);
			det = 1.0f / det;
			Vector2 x = new Vector2();
			x.X = det * (a22 * b.X - a12 * b.Y);
			x.Y = det * (a11 * b.Y - a21 * b.X);
			return x;
		}

		public Vector3 ex, ey, ez;
	}
}