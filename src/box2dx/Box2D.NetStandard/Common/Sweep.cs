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
	public struct Sweep
	{
		public Vector2 localCenter; //local center of mass position
		public Vector2 c0, c;       //local center of mass position
		public float a0, a;         //world angles
		public float alpha0;

		/// <summary>
		///  Get the interpolated transform at a specific time.
		/// </summary>
		/// <param name="alpha">Alpha is a factor in [0,1], where 0 indicates t0.</param>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public void GetTransform(out Transform xf, in float beta)
		{
			xf.p = c0 + beta * (c - c0);
			float angle = a0 + beta * (a - a0);
			xf.q = Matrex.CreateRotation(angle);
			xf.p -= Vector2.Transform(localCenter, xf.q); // Math.Mul(xf.q, localCenter);
		}

		/// <summary>
		///  Advance the sweep forward, yielding a new initial state.
		/// </summary>
		/// <param name="t">The new initial time.</param>
		public void Advance(float alpha)
		{
			//Debug.Assert(alpha0 < 1.0f);
			float beta = (alpha - alpha0) / (1.0f - alpha0);
			c0 += beta * (c - c0);
			a0 += beta * (a - a0);
			alpha0 = alpha;
		}

		public void Normalize()
		{
			float d = Settings.Tau * MathF.Floor(a0 / Settings.Tau);
			a0 -= d;
			a -= d;
		}
	}
}