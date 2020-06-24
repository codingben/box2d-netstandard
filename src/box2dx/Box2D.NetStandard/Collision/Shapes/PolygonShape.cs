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

//#define DEBUG

using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;
using Math = Box2D.NetStandard.Common.Math;

namespace Box2D.NetStandard.Collision.Shapes {
  /// <summary>
  /// A convex polygon. It is assumed that the interior of the polygon is to the left of each edge.
  /// </summary>
  public class PolygonShape : Shape {
    internal Vector2   m_centroid;
    internal Vector2[] m_vertices = new Vector2[Settings.MaxPolygonVertices];
    internal Vector2[] m_normals  = new Vector2[Settings.MaxPolygonVertices];
    internal int       m_count;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public PolygonShape() {
      m_type     = ShapeType.Polygon;
      m_radius   = Settings.PolygonRadius;
      m_count    = 0;
      m_centroid = Vector2.Zero;
    }

    public PolygonShape(params Vector2[] vectors) : this() {
      Set(vectors);
    }

    public PolygonShape(float hX, float hY) : this() {
      SetAsBox(hX,hY);
    }

    public override Shape Clone() => (Shape) MemberwiseClone();


    public void SetAsBox(float hx, float hy) {
      m_count       = 4;
      m_vertices[0] = new Vector2(-hx, -hy);
      m_vertices[1] = new Vector2(hx,  -hy);
      m_vertices[2] = new Vector2(hx,  hy);
      m_vertices[3] = new Vector2(-hx, hy);

      m_normals[0] = new Vector2(0,  -1);
      m_normals[1] = new Vector2(1,  0);
      m_normals[2] = new Vector2(0,  1);
      m_normals[3] = new Vector2(-1, 0);

      m_centroid = Vector2.Zero;
    }

    public void SetAsBox(float hx, float hy, in Vector2 center, float angle) {
      SetAsBox(hx, hy);
      m_centroid = center;

      Transform xf = new Transform();
      xf.p = center;
      xf.q = Matrix3x2.CreateRotation(angle);// .Set(angle);

      for (int i = 0; i < m_count; i++) {
        m_vertices[i] = Math.Mul(xf,   m_vertices[i]);
        m_normals[i]  = Math.Mul(xf.q, m_normals[i]);
      }
    }

    public override int GetChildCount() => 1;

    static Vector2 ComputeCentroid(in Vector2[] vs, int count) {
      //Debug.Assert(count >= 3);

      Vector2 c    = Vector2.Zero;
      float   area = 0.0f;

      // pRef is the reference point for forming triangles.
      // It's location doesn't change the result (except for rounding error).
      Vector2 pRef = Vector2.Zero;

      const float inv3 = 1.0f / 3.0f;

      for (int i = 0; i < count; ++i) {
        // Triangle vertices.
        Vector2 p1 = pRef;
        Vector2 p2 = vs[i];
        Vector2 p3 = i + 1 < count ? vs[i + 1] : vs[0];

        Vector2 e1 = p2 - p1;
        Vector2 e2 = p3 - p1;

        float D = Vectex.Cross(e1, e2);

        float triangleArea = 0.5f * D;
        area += triangleArea;

        // Area weighted centroid
        c += triangleArea * inv3 * (p1 + p2 + p3);
      }

      // Centroid
      //Debug.Assert(area > Settings.FLT_EPSILON);
      c *= 1.0f / area;
      return c;
    }


    public void Set(in Vector2[] vertices) {
      int count = vertices.Length;
      //Debug.Assert(3 <= count && count <= Settings.MaxPolygonVertices);
      if (count < 3) {
        SetAsBox(1f, 1f);
        return;
      }

      int n = System.Math.Min(count, Settings.MaxPolygonVertices);
      // Perform welding and copy vertices into local buffer.
      Vector2[] ps        = new Vector2[Settings.MaxPolygonVertices];
      int    tempCount = 0;
      for (int i = 0; i < n; ++i) {
        Vector2 v = vertices[i];

        bool unique = true;
        for (int j = 0; j < tempCount; ++j) {
          if (Vector2.DistanceSquared(v, ps[j]) < ((0.5f * Settings.LinearSlop) * (0.5f * Settings.LinearSlop))) {
            unique = false;
            break;
          }
        }

        if (unique) {
          ps[tempCount++] = v;
        }
      }

      n = tempCount;
      if (n < 3) {
        // Polygon is degenerate.
        //Debug.Assert(false);
        SetAsBox(1.0f, 1.0f);
        return;
      }

      // Create the convex hull using the Gift wrapping algorithm
      // http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

      // Find the right most point on the hull
      int i0 = 0;
      float x0 = ps[0].X;
      for (int i = 1; i < n; ++i) {
        float x = ps[i].X;
        if (x > x0 || (x == x0 && ps[i].Y < ps[i0].Y)) {
          i0 = i;
          x0 = x;
        }
      }

      int[] hull = new int[Settings.MaxPolygonVertices];
      int   m    = 0;
      int   ih   = i0;

      for (;;) {
        //Debug.Assert(m < Settings.MaxPolygonVertices);
        hull[m] = ih;

        int ie = 0;
        for (int j = 1; j < n; ++j) {
          if (ie == ih) {
            ie = j;
            continue;
          }

          Vector2 r = ps[ie] - ps[hull[m]];
          Vector2 v = ps[j]  - ps[hull[m]];
          float  c = Vectex.Cross(r, v);
          if (c < 0.0f) {
            ie = j;
          }

          // Collinearity check
          if (c == 0.0f && v.LengthSquared() > r.LengthSquared()) {
            ie = j;
          }
        }

        ++m;
        ih = ie;

        if (ie == i0) {
          break;
        }
      }

      if (m < 3) {
        // Polygon is degenerate.
        //Debug.Assert(false);
        SetAsBox(1.0f, 1.0f);
        return;
      }

      m_count = m;

      // Copy vertices.
      for (int i = 0; i < m; ++i) {
        m_vertices[i] = ps[hull[i]];
      }

      // Compute normals. Ensure the edges have non-zero length.
      for (int i = 0; i < m; ++i) {
        int  i1   = i;
        int  i2   = i + 1 < m ? i + 1 : 0;
        Vector2 edge = m_vertices[i2] - m_vertices[i1];
        //Debug.Assert(edge.LengthSquared() > Settings.FLT_EPSILON_SQUARED);
        m_normals[i] = Vector2.Normalize(Vectex.Cross(edge, 1.0f));
      }

      // Compute the polygon centroid.
      m_centroid = ComputeCentroid(m_vertices, m);
    }

    public override bool TestPoint(in Transform xf, in Vector2 p) {
      Vector2 pLocal = Math.MulT(xf.q, p - xf.p);

      for (int i = 0; i < m_count; ++i) {
        float dot = Vector2.Dot(m_normals[i], pLocal - m_vertices[i]);
        if (dot > 0.0f) {
          return false;
        }
      }

      return true;
    }

    public override bool RayCast(out RayCastOutput output, in RayCastInput input, in Transform xf,
      int                                          childIndex) {
      output = default;
      // Put the ray into the polygon's frame of reference.
      Vector2 p1 = Math.MulT(xf.q, input.p1 - xf.p);
      Vector2 p2 = Math.MulT(xf.q, input.p2 - xf.p);
      Vector2 d  = p2 - p1;

      float lower = 0.0f, upper = input.maxFraction;

      int index = -1;

      for (int i = 0; i < m_count; ++i) {
        // p = p1 + a * d
        // dot(normal, p - v) = 0
        // dot(normal, p1 - v) + a * dot(normal, d) = 0
        float numerator   = Vector2.Dot(m_normals[i], m_vertices[i] - p1);
        float denominator = Vector2.Dot(m_normals[i], d);

        if (denominator == 0.0f) {
          if (numerator < 0.0f) {
            return false;
          }
        }
        else {
          // Note: we want this predicate without division:
          // lower < numerator / denominator, where denominator < 0
          // Since denominator < 0, we have to flip the inequality:
          // lower < numerator / denominator <==> denominator * lower > numerator.
          if (denominator < 0.0f && numerator < lower * denominator) {
            // Increase lower.
            // The segment enters this half-space.
            lower = numerator / denominator;
            index = i;
          }
          else if (denominator > 0.0f && numerator < upper * denominator) {
            // Decrease upper.
            // The segment exits this half-space.
            upper = numerator / denominator;
          }
        }

        // The use of epsilon here causes the assert on lower to trip
        // in some cases. Apparently the use of epsilon was to make edge
        // shapes work, but now those are handled separately.
        //if (upper < lower - b2_epsilon)
        if (upper < lower) {
          return false;
        }
      }

      //Debug.Assert(0.0f <= lower && lower <= input.maxFraction);

      if (index >= 0) {
        output.fraction = lower;
        output.normal   = Math.Mul(xf.q, m_normals[index]);
        return true;
      }

      return false;
    }

    public override void ComputeAABB(out AABB aabb, in Transform xf, int childIndex) {
      Vector2 lower = Math.Mul(xf, m_vertices[0]);
      Vector2 upper = lower;

      for (int i = 1; i < m_count; ++i) {
        Vector2 v = Math.Mul(xf, m_vertices[i]);
        lower = Vector2.Min(lower, v);
        upper = Vector2.Max(upper, v);
      }

      Vector2 r = new Vector2(m_radius, m_radius);
      aabb.lowerBound = lower - r;
      aabb.upperBound = upper + r;
    }

    public override void ComputeMass(out MassData massData, float density) {
      // Polygon mass, centroid, and inertia.
      // Let rho be the polygon density in mass per unit area.
      // Then:
      // mass = rho * int(dA)
      // centroid.x = (1/mass) * rho * int(x * dA)
      // centroid.y = (1/mass) * rho * int(y * dA)
      // I = rho * int((x*x + y*y) * dA)
      //
      // We can compute these integrals by summing all the integrals
      // for each triangle of the polygon. To evaluate the integral
      // for a single triangle, we make a change of variables to
      // the (u,v) coordinates of the triangle:
      // x = x0 + e1x * u + e2x * v
      // y = y0 + e1y * u + e2y * v
      // where 0 <= u && 0 <= v && u + v <= 1.
      //
      // We integrate u from [0,1-v] and then v from [0,1].
      // We also need to use the Jacobian of the transformation:
      // D = cross(e1, e2)
      //
      // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
      //
      // The rest of the derivation is handled by computer algebra.

      //Debug.Assert(m_count >= 3);

      var   center = Vector2.Zero;
      float area   = 0.0f;
      float I      = 0.0f;

      // s is the reference point for forming triangles.
      // It's location doesn't change the result (except for rounding error).
      Vector2 s = Vector2.Zero;

      // This code would put the reference point inside the polygon.
      for (int i = 0; i < m_count; ++i) {
        s += m_vertices[i];
      }

      s *= 1.0f / m_count;

      const float k_inv3 = 1.0f / 3.0f;

      for (int i = 0; i < m_count; ++i) {
        // Triangle vertices.
        Vector2 e1 = m_vertices[i] - s;
        Vector2 e2 = i + 1 < m_count ? m_vertices[i + 1] - s : m_vertices[0] - s;

        float D = Vectex.Cross(e1, e2);

        float triangleArea = 0.5f * D;
        area += triangleArea;

        // Area weighted centroid
        center += triangleArea * k_inv3 * (e1 + e2);

        float ex1 = e1.X, ey1 = e1.Y;
        float ex2 = e2.X, ey2 = e2.Y;

        float intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
        float inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

        I += (0.25f * k_inv3 * D) * (intx2 + inty2);
      }

      // Total mass
      massData.mass = density * area;

      // Center of mass
      //Debug.Assert(area > Settings.FLT_EPSILON);
      center          *= 1.0f / area;
      massData.center =  center + s;

      // Inertia tensor relative to the local origin (point s).
      massData.I = density * I;

      // Shift to center of mass then to original body origin.
      massData.I += massData.mass * (Vector2.Dot(massData.center, massData.center) - Vector2.Dot(center, center));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2[] GetVertices() => m_vertices;
  }
}