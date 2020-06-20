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
using int32 = System.Int32;
using b2Vec2 = System.Numerics.Vector2;

namespace Box2DX.Collision
{
	public class EdgeShape : Shape {
    internal Vector2 m_vertex1;
    internal Vector2 m_vertex2;

    internal Vector2 m_vertex0;
    internal Vector2 m_vertex3;
    internal bool m_hasVertex0;
    internal bool m_hasVertex3;

    public EdgeShape() {
      m_type = ShapeType.Edge;
      m_radius = Settings.PolygonRadius;
      m_vertex0 = Vector2.Zero;
      m_vertex3=Vector2.Zero;
      m_hasVertex0 = false;
      m_hasVertex3 = false;
    }

    public Vector2 Vertex1 {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get=>m_vertex1;
    }
    
    public Vector2 Vertex2 {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get=>m_vertex2;
    }

    public void Set(in Vector2 v1, in Vector2 v2) {
      m_vertex1    = v1;
      m_vertex2    = v2;
      m_hasVertex0 = false;
      m_hasVertex3 = false;
    }


    public override Shape Clone() {
      return (EdgeShape) MemberwiseClone();
    }

    public override int GetChildCount() => 1;

    public override bool TestPoint(in Transform xf, in Vector2 p) => false;

    public override bool RayCast(out RayCastOutput output, in RayCastInput input, in Transform xf, int childIndex) {
      output = default;
      // Put the ray into the edge's frame of reference.
      b2Vec2 p1 = Math.MulT(xf.q, input.p1 - xf.p);
      b2Vec2 p2 = Math.MulT(xf.q, input.p2 - xf.p);
      b2Vec2 d  = p2 - p1;

      b2Vec2 v1 = m_vertex1;
      b2Vec2 v2 = m_vertex2;
      b2Vec2 e  = v2 - v1;
      b2Vec2 normal = Vector2.Normalize(new Vector2(e.Y, -e.X));

      // q = p1 + t * d
      // dot(normal, q - v1) = 0
      // dot(normal, p1 - v1) + t * dot(normal, d) = 0
      float numerator   = Vector2.Dot(normal, v1 - p1);
      float denominator = Vector2.Dot(normal, d);

      if (denominator == 0.0f)
      {
        return false;
      }

      float t = numerator / denominator;
      if (t < 0.0f || input.maxFraction < t)
      {
        return false;
      }

      b2Vec2 q = p1 + t * d;

      // q = v1 + s * r
      // s = dot(q - v1, r) / dot(r, r)
      b2Vec2 r  = v2 - v1;
      float  rr = Vector2.Dot(r, r);
      if (rr == 0.0f)
      {
        return false;
      }

      float s = Vector2.Dot(q - v1, r) / rr;
      if (s < 0.0f || 1.0f < s)
      {
        return false;
      }

      output.fraction = t;
      if (numerator > 0.0f)
      {
        output.normal = -Math.Mul(xf.q, normal);
      }
      else
      {
        output.normal = Math.Mul(xf.q, normal);
      }
      return true;
    }

    public override void ComputeAABB(out AABB aabb, in Transform xf, int childIndex) {
      b2Vec2 v1 = Math.Mul(xf, m_vertex1);
      b2Vec2 v2 = Math.Mul(xf, m_vertex2);

      b2Vec2 lower = Vector2.Min(v1, v2);
      b2Vec2 upper = Vector2.Max(v1, v2);

      b2Vec2 r = new Vector2(m_radius, m_radius);
      aabb.lowerBound = lower - r;
      aabb.upperBound = upper + r;
    }

    public override void ComputeMass(out MassData massData, float density) {
      massData.mass   = 0.0f;
      massData.center = 0.5f * (m_vertex1 + m_vertex2);
      massData.I      = 0.0f;
    }
  }
}
