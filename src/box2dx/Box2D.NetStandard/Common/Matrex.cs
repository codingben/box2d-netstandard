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

namespace Box2D.NetStandard.Common
{
	/// <summary>
	///  Matrix extension methods
	/// </summary>
	public static class Matrex
	{
		/// <summary>
		///  Solve A * x = b, where b is a column vector. This is more efficient
		///  than computing the inverse in one-shot cases.
		/// </summary>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Vector2 Solve(this Matrix3x2 m, Vector2 b)
		{
			float det = 1f / m.GetDeterminant();
			return new Vector2(
			                   det * (m.M22 * b.X - m.M12 * b.Y),
			                   det * (m.M11 * b.Y - m.M21 * b.X));
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Matrix3x2 CreateRotation(float angle)
		{
			float cos = MathF.Cos(angle);
			float sin = MathF.Sin(angle);

			Matrix3x2 result = Matrix3x2.Identity;
			result.M11 = cos;
			result.M12 = sin;
			result.M21 = -sin;
			result.M22 = cos;
			return result;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static void Invert(in Matrix3x2 matrix, out Matrix3x2 result)
		{
			float x = matrix.M11 * matrix.M22 - matrix.M21 * matrix.M12;
			float num = 1f / x;
			result.M11 = matrix.M22 * num;
			result.M12 = -matrix.M12 * num;
			result.M21 = -matrix.M21 * num;
			result.M22 = matrix.M11 * num;
			result.M31 = result.M32 = 0;
		}
	}
}