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

namespace Box2D.NetStandard.Common
{
	/// <summary>
	/// A 2D column vector with 3 elements.
	/// </summary>
	public struct Vec3
	{
		/// <summary>
		/// Construct using coordinates.
		/// </summary>
		public Vec3(float x, float y, float z) { X = x; Y = y; Z = z; }

		/// <summary>
		/// Set this vector to all zeros.
		/// </summary>
		public void SetZero() { X = 0.0f; Y = 0.0f; Z = 0.0f; }

		/// <summary>
		/// Set this vector to some specified coordinates.
		/// </summary>
		public void Set(float x, float y, float z) { X = x; Y = y; Z = z; }

		/// <summary>
		/// Perform the dot product on two vectors.
		/// </summary>
		public static float Dot(Vec3 a, Vec3 b)
		{
			return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
		}

		/// <summary>
		/// Perform the cross product on two vectors.
		/// </summary>
		public static Vec3 Cross(Vec3 a, Vec3 b)
		{
			return new Vec3(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
		}

		/// <summary>
		/// Negate this vector.
		/// </summary>
		public static Vec3 operator -(Vec3 v)
		{
			return new Vec3(-v.X, -v.Y, -v.Z);
		}

		/// <summary>
		/// Add two vectors component-wise.
		/// </summary>
		public static Vec3 operator +(Vec3 v1, Vec3 v2)
		{
			return new Vec3(v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z);
		}

		/// <summary>
		/// Subtract two vectors component-wise.
		/// </summary>
		public static Vec3 operator -(Vec3 v1, Vec3 v2)
		{
			return new Vec3(v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z);
		}

		/// <summary>
		/// Multiply this vector by a scalar.
		/// </summary>
		public static Vec3 operator *(Vec3 v, float s)
		{
			return new Vec3(v.X * s, v.Y * s, v.Z * s);
		}

		/// <summary>
		/// Multiply this vector by a scalar.
		/// </summary>
		public static Vec3 operator *(float s, Vec3 v)
		{
			return new Vec3(v.X * s, v.Y * s, v.Z * s);
		}

		public float X, Y, Z;
	}
}