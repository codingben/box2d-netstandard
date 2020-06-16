/*
  Box2DX Copyright (c) 2009 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com

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
using System.Runtime.CompilerServices;
using Box2DX.Common;
using Math = Box2DX.Common.Math;

namespace Box2DX.Collision
{
	public class EdgeShape : Shape
	{
		public Vector2 _v1;
		public Vector2 _v2;

		public float _length;

		public Vector2 _normal;

		public Vector2 _direction;

		// Unit vector halfway between m_direction and m_prevEdge.m_direction:
		public Vector2 _cornerDir1;

		// Unit vector halfway between m_direction and m_nextEdge.m_direction:
		public Vector2 _cornerDir2;

		public bool _cornerConvex1;
		public bool _cornerConvex2;

		public EdgeShape _nextEdge;
		public EdgeShape _prevEdge;

		public EdgeShape()
		{
			_type = ShapeType.EdgeShape;
			_radius = Settings.PolygonRadius;
		}

		public override void Dispose()
		{
			if (_prevEdge != null)
			{
				_prevEdge._nextEdge = null;
			}

			if (_nextEdge != null)
			{
				_nextEdge._prevEdge = null;
			}
		}

		public void Set(Vector2 v1, Vector2 v2)
		{
			_v1 = v1;
			_v2 = v2;

			_direction = _v2 - _v1;
			_length = _direction.Length();
			_direction=Vector2.Normalize(_direction);
			_normal = Vectex.Cross(_direction, 1.0f);

			_cornerDir1 = _normal;
			_cornerDir2 = -1.0f * _normal;
		}

		public override bool TestPoint(XForm transform, Vector2 p) => false;

		public override SegmentCollide TestSegment(XForm transform, out float lambda, out Vector2 normal, Segment segment, float maxLambda)
		{
			Vector2 r = segment.P2 - segment.P1;
			Vector2 v1 = Math.Mul(transform, _v1);
			Vector2 d = Math.Mul(transform, _v2) - v1;
			Vector2 n = Vectex.Cross(d, 1.0f);

			float k_slop = 100.0f * Settings.FLT_EPSILON;
			float denom = -Vector2.Dot(r, n);

			// Cull back facing collision and ignore parallel segments.
			if (denom > k_slop)
			{
				// Does the segment intersect the infinite line associated with this segment?
				Vector2 b = segment.P1 - v1;
				float a = Vector2.Dot(b, n);

				if (0.0f <= a && a <= maxLambda * denom)
				{
					float mu2 = -r.X * b.Y + r.Y * b.X;

					// Does the segment intersect this segment?
					if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0f + k_slop))
					{
						a /= denom;
						n = Vector2.Normalize(n);
						lambda = a;
						normal = n;
						return SegmentCollide.HitCollide;
					}
				}
			}

			lambda = 0;
			normal = new Vector2();
			return SegmentCollide.MissCollide;
		}

		public override void ComputeAABB(out AABB aabb, XForm transform)
		{
			Vector2 v1 = Math.Mul(transform, _v1);
			Vector2 v2 = Math.Mul(transform, _v2);

			Vector2 r = new Vector2(_radius, _radius);
			aabb.LowerBound = Vector2.Min(v1, v2) - r;
			aabb.UpperBound = Vector2.Max(v1, v2) + r;
		}

		public override void ComputeMass(out MassData massData, float density)
		{
			massData.Mass = 0.0f;
			massData.Center = _v1;
			massData.I = 0.0f;
		}

		public void SetPrevEdge(EdgeShape edge, Vector2 cornerDir, bool convex)
		{
			_prevEdge = edge;
			_cornerDir1 = cornerDir;
			_cornerConvex1 = convex;
		}

		public void SetNextEdge(EdgeShape edge, Vector2 cornerDir, bool convex)
		{
			_nextEdge = edge;
			_cornerDir2 = cornerDir;
			_cornerConvex2 = convex;
		}

		public override float ComputeSubmergedArea(Vector2 normal, float offset, XForm xf, out Vector2 c)
		{
			//Note that v0 is independent of any details of the specific edge
			//We are relying on v0 being consistent between multiple edges of the same body
			Vector2 v0 = offset * normal;
			//b2Vector2 v0 = xf.position + (offset - b2Dot(normal, xf.position)) * normal;

			Vector2 v1 = Math.Mul(xf, _v1);
			Vector2 v2 = Math.Mul(xf, _v2);

			float d1 = Vector2.Dot(normal, v1) - offset;
			float d2 = Vector2.Dot(normal, v2) - offset;

			if (d1 > 0.0f)
			{
				if (d2 > 0.0f)
				{
					c = new Vector2();
					return 0.0f;
				}
				else
				{
					v1 = -d2 / (d1 - d2) * v1 + d1 / (d1 - d2) * v2;
				}
			}
			else
			{
				if (d2 > 0.0f)
				{
					v2 = -d2 / (d1 - d2) * v1 + d1 / (d1 - d2) * v2;
				}
				else
				{
					//Nothing
				}
			}

			// v0,v1,v2 represents a fully submerged triangle
			float k_inv3 = 1.0f / 3.0f;

			// Area weighted centroid
			c = k_inv3 * (v0 + v1 + v2);

			Vector2 e1 = v1 - v0;
			Vector2 e2 = v2 - v0;

			return 0.5f * Vectex.Cross(e1, e2);
		}

		public float Length
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => _length;
		}

		public Vector2 Vertex1
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => _v1;
		}

		public Vector2 Vertex2
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => _v2;
		}

		public Vector2 NormalVector
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => _normal;
		}

		public Vector2 DirectionVector
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => _direction;
		}

		public Vector2 Corner1Vector
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => _cornerDir1;
		}

		public Vector2 Corner2Vector
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => _cornerDir2;
		}

		public override int GetSupport(Vector2 d)
		{
			return Vector2.Dot(_v1, d) > Vector2.Dot(_v2, d) ? 0 : 1;
		}

		public override Vector2 GetSupportVertex(Vector2 d)
		{
			return Vector2.Dot(_v1, d) > Vector2.Dot(_v2, d) ? _v1 : _v2;
		}

		public override Vector2 GetVertex(int index)
		{
			Debug.Assert(0 <= index && index < 2);
			if (index == 0) return _v1;
			else return _v2;
		}

		public bool Corner1IsConvex
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => _cornerConvex1;
		}

		public bool Corner2IsConvex
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => _cornerConvex2;
		}

		public override float ComputeSweepRadius(Vector2 pivot)
		{
			float ds1 = Vector2.DistanceSquared(_v1, pivot);
			float ds2 = Vector2.DistanceSquared(_v2, pivot);
			return MathF.Sqrt(MathF.Max(ds1, ds2));
		}
	}
}
