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
using System.Runtime.InteropServices;

namespace Box2D.NetStandard.Common
{
	internal static class Math
	{
		internal const ushort USHRT_MAX = ushort.MaxValue;
		internal const byte UCHAR_MAX = byte.MaxValue;
		internal const int RAND_LIMIT = 32767;

		private static readonly Random s_rnd = new Random();

		/// <summary>
		///  This function is used to ensure that a floating point number is
		///  not a NaN or infinity.
		/// </summary>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static bool IsValid(float x) =>
			!(float.IsNaN(x) || float.IsNegativeInfinity(x) || float.IsPositiveInfinity(x));

		/// <summary>
		///  This is a approximate yet fast inverse square-root.
		/// </summary>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static float InvSqrt(float x)
		{
			Convert convert = default;
			convert.x = x;
			float xhalf = 0.5f * x;
			convert.i = 0x5f3759df - (convert.i >> 1);
			x = convert.x;
			x = x * (1.5f - xhalf * x * x);
			return x;
		}

		[Obsolete("Use MathF.Sqrt", true)]
		internal static float Sqrt(float x) => (float) System.Math.Sqrt(x);

		/// <summary>
		///  Random number in range [-1,1]
		/// </summary>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static float Random()
		{
			float r = s_rnd.Next() & RAND_LIMIT;
			r /= RAND_LIMIT;
			r = 2.0f * r - 1.0f;
			return r;
		}

		/// <summary>
		///  Random floating point number in range [lo, hi]
		/// </summary>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static float Random(float lo, float hi)
		{
			float r = s_rnd.Next() & RAND_LIMIT;
			r /= RAND_LIMIT;
			r = (hi - lo) * r + lo;
			return r;
		}

		/// <summary>
		///  "Next Largest Power of 2
		///  Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
		///  that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
		///  the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
		///  largest power of 2. For a 32-bit value:"
		/// </summary>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static uint NextPowerOfTwo(uint x)
		{
			x |= x >> 1;
			x |= x >> 2;
			x |= x >> 4;
			x |= x >> 8;
			x |= x >> 16;
			return x + 1;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static bool IsPowerOfTwo(uint x)
		{
			bool result = x > 0 && (x & (x - 1)) == 0;
			return result;
		}

		/// <summary>
		///  Multiply a matrix transpose times a vector. If a rotation matrix is provided,
		///  then this transforms the vector from one frame to another (inverse transform).
		/// </summary>
		// [MethodImpl(MethodImplOptions.AggressiveInlining)]
		// internal static Vector2 MulT(Mat22 A, Vector2 v) => new Vector2(Vector2.Dot(v, A.ex), Vector2.Dot(v, A.ey));
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static Vector2 MulT(Matrix3x2 A, Vector2 v)
		{
			/*Matrix3x2*/
			Matrex.Invert(A, out Matrix3x2 AT);
			return Vector2.Transform(v, AT);
		}


		/// <summary>
		///  A * B
		/// </summary>
		// [MethodImpl(MethodImplOptions.AggressiveInlining)]
		// internal static Mat22 Mul(Mat22 A, Mat22 B) {
		//   Mat22 C = new Mat22();
		//   C.Set(Mul(A, B.ex), Mul(A, B.ey));
		//   return C;
		// }
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static Matrix3x2 Mul(Matrix3x2 A, Matrix3x2 B) => A * B;

		/// <summary>
		///  A^T * B
		/// </summary>
		// [MethodImpl(MethodImplOptions.AggressiveInlining)]
		// internal static Mat22 MulT(Mat22 A, Mat22 B) {
		//   Vector2 c1 = new Vector2(Vector2.Dot(A.ex, B.ex), Vector2.Dot(A.ey, B.ex));
		//   Vector2 c2 = new Vector2(Vector2.Dot(A.ex, B.ey), Vector2.Dot(A.ey, B.ey));
		//   Mat22   C  = new Mat22();
		//   C.Set(c1, c2);
		//   return C;
		// }
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static Matrix3x2 MulT(Matrix3x2 A, Matrix3x2 B)
		{
			/*Matrix3x2*/
			Matrex.Invert(A, out Matrix3x2 AT);
			return AT * B;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static Vector2 Mul(Transform T, Vector2 v) => T.p + Vector2.Transform(v, T.q); //Mul(T.q, v);

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static Vector2 MulT(Transform T, Vector2 v) => MulT(T.q, v - T.p);

		/// <summary>
		///  Multiply a matrix times a vector.
		/// </summary>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static Vector3 Mul(Mat33 A, Vector3 v) => v.X * A.ex + v.Y * A.ey + v.Z * A.ez;


		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static Vector2 Mul(Rot q, Vector2 v) => new Vector2(q.c * v.X - q.s * v.Y, q.s * v.X + q.c * v.Y);

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static Vector2 MulT(Rot q, Vector2 v) => new Vector2(q.c * v.X + q.s * v.Y, -q.s * v.X + q.c * v.Y);


		// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
		//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static Transform Mul(in Transform A, in Transform B)
		{
			Transform C;
			C.q = Mul(A.q, B.q);
			C.p = A.p + Vector2.Transform(B.p, A.q); //Mul(A.q, B.p);
			return C;
		}

		// v2 = A.q' * (B.q * v1 + B.p - A.p)
		//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static Transform MulT(in Transform A, in Transform B)
		{
			Transform C;
			C.q = MulT(A.q, B.q);
			C.p = MulT(A.q, B.p - A.p);
			return C;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		internal static Vector2 Mul22(in Mat33 A, in Vector2 v) =>
			new Vector2(A.ex.X * v.X + A.ey.X * v.X, A.ex.Y * v.X + A.ey.Y * v.Y);

		[StructLayout(LayoutKind.Explicit)]
		internal struct Convert
		{
			[FieldOffset(0)]
			public float x;

			[FieldOffset(0)]
			public int i;
		}
	}
}