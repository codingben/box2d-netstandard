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

namespace Box2D.NetStandard.Common
{
	/// <summary>
	///  A 2D column vector.
	/// </summary>
	[Obsolete("Since Vec2 has been replaced with System.Numerics.Vector2, this will be implictly cast to a Vector2. It is recommended to change your code to use System.Numerics.Vector2 instead.")]
	public struct Vec2
	{
		private bool Equals(Vec2 other) => X.Equals(other.X) && Y.Equals(other.Y);

		public override bool Equals(object obj) => obj is Vec2 other && Equals(other);

		public override int GetHashCode() => HashCode.Combine(X, Y);

		[Obsolete("Warning: Implicit cast from Vec2 to System.Numerics.Vector2. You are advised to change your code to expect Vector2.")]
		public static implicit operator Vector2(Vec2 src) => new Vector2(src.X, src.Y);

		[Obsolete("Warning: Implicit cast from System.Numerics.Vector2 to Vec2. You are advised to change your code to expect Vector2.")]
		public static implicit operator Vec2(Vector2 src) => new Vec2(src.X, src.Y);

		[Obsolete("Warning: Implicit cast from System.Numerics.Vector2 to Vec2. You are advised to change your code to expect Vector2.")]
		public static implicit operator Vec2((float, float) src) => new Vec2(src.Item1, src.Item2);

		public float X, Y;

		/// <summary>
		///  Construct using coordinates.
		/// </summary>
		[Obsolete("Since Vec2 has been replaced with System.Numerics.Vector2, this will be implictly cast to a Vector2. It is recommended to change your code to use System.Numerics.Vector2 instead.")]
		public Vec2(float x)
		{
			X = x;
			Y = x;
		}

		/// <summary>
		///  Construct using coordinates.
		/// </summary>
		[Obsolete("Since Vec2 has been replaced with System.Numerics.Vector2, this will be implictly cast to a Vector2. It is recommended to change your code to use System.Numerics.Vector2 instead.")]
		public Vec2(float x, float y)
		{
			X = x;
			Y = y;
		}

		/// <summary>
		///  Set this vector to all zeros.
		/// </summary>
		[Obsolete("Since Vec2 has been replaced with System.Numerics.Vector2, this means vectors are now considered immutable. Instead, please create a new Vector2 and assign it.",
		          true)]
		public void SetZero()
		{
			X = 0.0f;
			Y = 0.0f;
		}

		/// <summary>
		///  Set this vector to some specified coordinates.
		/// </summary>
		[Obsolete("Since Vec2 has been replaced with System.Numerics.Vector2, this means vectors are now considered immutable. Instead, please create a new Vector2 and assign it.",
		          true)]
		public void Set(float x, float y)
		{
			X = x;
			Y = y;
		}

		[Obsolete("Since Vec2 has been replaced with System.Numerics.Vector2, this means vectors are now considered immutable. Instead, please create a new Vector2 and assign it.",
		          true)]
		public void Set(float xy)
		{
			X = xy;
			Y = xy;
		}

		/// <summary>
		///  Get the length of this vector (the norm).
		/// </summary>
		[Obsolete("This will still work, but may be removed in a future version. Check the field or property and see if a newer Vector2 is available.")]
		public float Length() => (float) System.Math.Sqrt(X * X + Y * Y);

		/// <summary>
		///  Get the length squared. For performance, use this instead of
		///  Length (if possible).
		/// </summary>
		/// [Obsolete("This will still work, but may be removed in a future version. Check the field or property and see if a newer Vector2 is available.")]
		public float LengthSquared() => X * X + Y * Y;

		/// <summary>
		///  Convert this vector into a unit vector. Returns the length.
		/// </summary>
		[Obsolete("Since Vec2 has been replaced with System.Numerics.Vector2, this won't work any more. If you need the Length, get .Length. If you need to normalize a vector, call Vector2.Normalize and re-assign the result.",
		          true)]
		public float Normalize()
		{
			float length = Length();
			if (length < Settings.FLT_EPSILON)
			{
				return 0.0f;
			}

			float invLength = 1.0f / length;
			X *= invLength;
			Y *= invLength;

			return length;
		}

		/// <summary>
		///  Does this vector contain finite coordinates?
		/// </summary>
		[Obsolete("Please switch to System.Numerics.Vector2 and use Vector2.IsValid() instead.")]
		public bool IsValid
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => Math.IsValid(X) && Math.IsValid(Y);
		}

		/// <summary>
		///  Negate this vector.
		/// </summary>
		[Obsolete("Please switch to System.Numerics.Vector2.")]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Vec2 operator -(Vec2 v1) => new Vec2(-v1.X, -v1.Y);

		[Obsolete("Please switch to System.Numerics.Vector2.")]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Vec2 operator +(Vec2 v1, Vec2 v2) => new Vec2(v1.X + v2.X, v1.Y + v2.Y);

		[Obsolete("Please switch to System.Numerics.Vector2.")]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Vec2 operator -(Vec2 v1, Vec2 v2) => new Vec2(v1.X - v2.X, v1.Y - v2.Y);

		[Obsolete("Please switch to System.Numerics.Vector2.")]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Vec2 operator *(Vec2 v1, float a) => new Vec2(v1.X * a, v1.Y * a);

		[Obsolete("Please switch to System.Numerics.Vector2.")]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Vec2 operator *(float a, Vec2 v1) => new Vec2(v1.X * a, v1.Y * a);

		[Obsolete("Please switch to System.Numerics.Vector2.")]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static bool operator ==(Vec2 a, Vec2 b) => a.X == b.X && a.Y == b.Y;

		[Obsolete("Please switch to System.Numerics.Vector2.")]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static bool operator !=(Vec2 a, Vec2 b) => a.X != b.X || a.Y != b.Y;

		[Obsolete("Please switch to System.Numerics.Vector2.")]
		public static Vec2 Zero
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => new Vec2(0, 0);
		}


		/// <summary>
		///  Peform the dot product on two vectors.
		/// </summary>
		[Obsolete("Please switch to System.Numerics.Vector2.")]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static float Dot(Vec2 a, Vec2 b) => a.X * b.X + a.Y * b.Y;

		/// <summary>
		///  Perform the cross product on two vectors. In 2D this produces a scalar.
		/// </summary>
		[Obsolete("Please switch to System.Numerics.Vector2.")]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static float Cross(Vec2 a, Vec2 b) => a.X * b.Y - a.Y * b.X;

		/// <summary>
		///  Perform the cross product on a vector and a scalar.
		///  In 2D this produces a vector.
		/// </summary>
		[Obsolete("Please switch to System.Numerics.Vector2 and use Vectex.Cross.")]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Vec2 Cross(Vec2 a, float s) => new Vec2(s * a.Y, -s * a.X);

		/// <summary>
		///  Perform the cross product on a scalar and a vector.
		///  In 2D this produces a vector.
		/// </summary>
		[Obsolete("Please switch to System.Numerics.Vector2 and use Vectex.Cross.")]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Vec2 Cross(float s, Vec2 a) => new Vec2(-s * a.Y, s * a.X);

		[Obsolete("Use Vector2.Distance instead")]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static float Distance(Vec2 a, Vec2 b) => (a - b).Length();

		[Obsolete("Use Vector2.DistanceSquared instead")]
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static float DistanceSquared(Vec2 a, Vec2 b)
		{
			Vec2 c = a - b;
			return Dot(c, c);
		}

		internal static Vec2[] ConvertArray(Vector2[] vertices)
		{
			var result = new Vec2[vertices.Length];
			for (var i = 0; i < vertices.Length; i++)
			{
				result[i] = vertices[i];
			}

			return result;
		}
	}
}