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
using System.Diagnostics;
using System.Numerics;

namespace Box2D.NetStandard.Common
{
	public struct Sweep
	{
		public Vector2 localCenter;	//local center of mass position
		public Vector2 c0, c; //local center of mass position
		public float a0, a; //world angles
		//public float T0; //time interval = [T0,1], where T0 is in [0,1]
		public float alpha0;

		/// <summary>
		/// Get the interpolated transform at a specific time.
		/// </summary>
		/// <param name="alpha">Alpha is a factor in [0,1], where 0 indicates t0.</param>
		public void GetTransform(out Transform xf, float alpha)
		{
			xf = new Transform();
			xf.p = (1.0f - alpha) * c0 + alpha * c;
			float angle = (1.0f - alpha) * a0 + alpha * a;
			xf.q.Set(angle);

			// Shift to origin
			xf.p -= Math.Mul(xf.q, localCenter);
		}

		/// <summary>
		/// Advance the sweep forward, yielding a new initial state.
		/// </summary>
		/// <param name="t">The new initial time.</param>
		public void Advance(float alpha)
		{
			Debug.Assert(alpha0 < 1.0f);
			float beta = (alpha - alpha0) / (1.0f - alpha0);
			c0     += beta * (c - c0);
			a0     += beta * (a - a0);
			alpha0 =  alpha;
		}

		public void Normalize() {
			float twoPi = 2f * Settings.Pi;
			float d = twoPi * MathF.Floor(a0 / twoPi);
			a0 -= d;
			a -= d;
		}
	}
}
