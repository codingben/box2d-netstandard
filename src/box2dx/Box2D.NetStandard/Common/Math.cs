/*
  Box2DX Copyright (c) 2008 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com

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

using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Box2D.NetStandard.Common {
  internal class Math {
    internal const ushort USHRT_MAX  = ushort.MaxValue;
    internal const byte   UCHAR_MAX  = byte.MaxValue;
    internal const int    RAND_LIMIT = 32767;

    /// <summary>
    /// This function is used to ensure that a floating point number is
    /// not a NaN or infinity.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsValid(float x) {
      return !(float.IsNaN(x) || float.IsNegativeInfinity(x) || float.IsPositiveInfinity(x));
    }

    [System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Explicit)]
    internal struct Convert {
      [System.Runtime.InteropServices.FieldOffset(0)]
      public float x;

      [System.Runtime.InteropServices.FieldOffset(0)]
      public int i;
    }

    /// <summary>
    /// This is a approximate yet fast inverse square-root.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static float InvSqrt(float x) {
      Convert convert = new Convert();
      convert.x = x;
      float xhalf = 0.5f * x;
      convert.i = 0x5f3759df - (convert.i >> 1);
      x         = convert.x;
      x         = x * (1.5f - xhalf * x * x);
      return x;
    }

    private static Random s_rnd = new Random();

    /// <summary>
    /// Random number in range [-1,1]
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static float Random() {
      float r = (float) (s_rnd.Next() & RAND_LIMIT);
      r /= RAND_LIMIT;
      r =  2.0f * r - 1.0f;
      return r;
    }

    /// <summary>
    /// Random floating point number in range [lo, hi]
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static float Random(float lo, float hi) {
      float r = (float) (s_rnd.Next() & RAND_LIMIT);
      r /= RAND_LIMIT;
      r =  (hi - lo) * r + lo;
      return r;
    }

    /// <summary>
    /// "Next Largest Power of 2
    /// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
    /// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
    /// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
    /// largest power of 2. For a 32-bit value:"
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static uint NextPowerOfTwo(uint x) {
      x |= (x >> 1);
      x |= (x >> 2);
      x |= (x >> 4);
      x |= (x >> 8);
      x |= (x >> 16);
      return x + 1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsPowerOfTwo(uint x) {
      bool result = x > 0 && (x & (x - 1)) == 0;
      return result;
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Mat22 Abs(Mat22 A) => new Mat22(Vector2.Abs(A.ex), Vector2.Abs(A.ey));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Swap<T>(ref T a, ref T b) {
      T tmp = a;
      a = b;
      b = tmp;
    }

    /// <summary>
    /// Multiply a matrix times a vector. If a rotation matrix is provided,
    /// then this transforms the vector from one frame to another.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Vector2 Mul(Mat22 A, Vector2 v) =>
      new Vector2(A.ex.X * v.X + A.ey.X * v.Y, A.ex.Y * v.X + A.ey.Y * v.Y);

    /// <summary>
    /// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
    /// then this transforms the vector from one frame to another (inverse transform).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Vector2 MulT(Mat22 A, Vector2 v) => new Vector2(Vector2.Dot(v, A.ex), Vector2.Dot(v, A.ey));

    /// <summary>
    /// A * B
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Mat22 Mul(Mat22 A, Mat22 B) {
      Mat22 C = new Mat22();
      C.Set(Mul(A, B.ex), Mul(A, B.ey));
      return C;
    }

    /// <summary>
    /// A^T * B
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Mat22 MulT(Mat22 A, Mat22 B) {
      Vector2 c1 = new Vector2(Vector2.Dot(A.ex, B.ex), Vector2.Dot(A.ey, B.ex));
      Vector2 c2 = new Vector2(Vector2.Dot(A.ex, B.ey), Vector2.Dot(A.ey, B.ey));
      Mat22   C  = new Mat22();
      C.Set(c1, c2);
      return C;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Vector2 Mul(Transform T, Vector2 v) => T.p + Mul(T.q, v);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Vector2 MulT(Transform T, Vector2 v) => MulT(T.q, v - T.p);

    /// <summary>
    /// Multiply a matrix times a vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Vec3 Mul(Mat33 A, Vec3 v) => v.X * A.ex + v.Y * A.ey + v.Z * A.ez;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Vector2 Mul(Rot q, Vector2 v) =>
      new Vector2(q.c * v.X - q.s * v.Y,
                  q.s * v.X + q.c * v.Y);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Vector2 MulT(Rot q, Vector2 v) =>
      new Vector2(q.c  * v.X + q.s * v.Y,
                  -q.s * v.X + q.c * v.Y);


    // v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Transform Mul(in Transform A, in Transform B) {
      Transform C;
      C.q = Mul(A.q, B.q);
      C.p = Mul(A.q, B.p) + A.p;
      return C;
    }

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Transform MulT(in Transform A, in Transform B) {
      Transform C;
      C.q = MulT(A.q, B.q);
      C.p = MulT(A.q, B.p - A.p);
      return C;
    }
  }
}