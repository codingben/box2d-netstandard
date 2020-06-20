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
using Box2DX.Common;
using b2Vec2 = System.Numerics.Vector2;
using int32 = System.Int32;
using Math = Box2DX.Common.Math;
using uint8 = System.Byte;

namespace Box2DX.Collision {
  public partial class Collision {
    // This implements 2-sided edge vs circle collision.
    internal static void CollideEdgeAndCircle(out Manifold    manifold, in EdgeShape edgeA, in Transform xfA,
      in                                        CircleShape circleB,  in Transform xfB) {
      manifold = new Manifold();

      manifold.pointCount = 0;

      Vector2 Q = Math.MulT(xfA, Math.Mul(xfB, circleB.m_p));

      Vector2 A = edgeA.m_vertex1, B = edgeA.m_vertex2;
      Vector2 e = B - A;

      float u = Vector2.Dot(e, B - Q);
      float v = Vector2.Dot(e, Q - A);

      float radius = edgeA.m_radius + circleB.m_radius;

      ContactFeature cf = new ContactFeature();
      cf.indexB = 0;
      cf.typeB  = (byte) ContactFeatureType.Vertex;
      

      // Region A
      if (v <= 0.0f) {
        b2Vec2 P  = A;
        b2Vec2 d  = Q - P;
        float  dd = Vector2.Dot(d, d);
        if (dd > radius * radius) {
          return;
        }

        // Is there an edge connected to A?
        if (edgeA.m_hasVertex0) {
          b2Vec2 A1 = edgeA.m_vertex0;
          b2Vec2 B1 = A;
          b2Vec2 e1 = B1 - A1;
          float  u1 = Vector2.Dot(e1, B1 - Q);

          // Is the circle in Region AB of the previous edge?
          if (u1 > 0.0f) {
            return;
          }
        }

        cf.indexA                     = 0;
        cf.typeA                      = (byte) ContactFeatureType.Vertex;
        manifold.pointCount           = 1;
        manifold.type                 = ManifoldType.Circles;
        manifold.localNormal          = Vector2.Zero;
        manifold.localPoint           = P;
        manifold.points[0] = new ManifoldPoint();
        manifold.points[0].id.key     = 0;
        manifold.points[0].id.cf      = cf;
        manifold.points[0].localPoint = circleB.m_p;
        return;
      }

      // Region B
      if (u <= 0.0f) {
        b2Vec2 P  = B;
        b2Vec2 d  = Q - P;
        float  dd = Vector2.Dot(d, d);
        if (dd > radius * radius) {
          return;
        }

        // Is there an edge connected to B?
        if (edgeA.m_hasVertex3) {
          b2Vec2 B2 = edgeA.m_vertex3;
          b2Vec2 A2 = B;
          b2Vec2 e2 = B2 - A2;
          float  v2 = Vector2.Dot(e2, Q - A2);

          // Is the circle in Region AB of the next edge?
          if (v2 > 0.0f) {
            return;
          }
        }

        cf.indexA                     = 1;
        cf.typeA                      = (byte) ContactFeatureType.Vertex;
        manifold.pointCount           = 1;
        manifold.type                 = ManifoldType.Circles;
        manifold.localNormal          = Vector2.Zero;
        manifold.localPoint           = P;
        manifold.points[0] = new ManifoldPoint();
        manifold.points[0].id.key     = 0;
        manifold.points[0].id.cf      = cf;
        manifold.points[0].localPoint = circleB.m_p;
        return;
      }

      {
        // Region AB
        float den = Vector2.Dot(e, e);
        Debug.Assert(den > 0.0f);
        b2Vec2 P  = (1.0f / den) * (u * A + v * B);
        b2Vec2 d  = Q - P;
        float  dd = Vector2.Dot(d, d);
        if (dd > radius * radius) {
          return;
        }

        b2Vec2 n = new Vector2(-e.Y, e.X);
        if (Vector2.Dot(n, Q - A) < 0.0f) {
          n = new Vector2(-n.X, -n.Y);
        }

        n = Vector2.Normalize(n);

        cf.indexA                     = 0;
        cf.typeA                      = (byte) ContactFeatureType.Face;
        manifold.pointCount           = 1;
        manifold.type                 = ManifoldType.FaceA;
        manifold.localNormal          = n;
        manifold.localPoint           = A;
        manifold.points[0] = new ManifoldPoint();
        manifold.points[0].id.key     = 0;
        manifold.points[0].id.cf      = cf;
        manifold.points[0].localPoint = circleB.m_p;
      }
    }

    internal static void CollideEdgeAndPolygon(out Manifold     manifold,
      in                           EdgeShape    edgeA,    in Transform xfA,
      in                           PolygonShape polygonB, in Transform xfB) {
      EPCollider collider = new EPCollider();
      collider.Collide(out manifold, edgeA, xfA, polygonB, xfB);
    }
  }

  // This structure is used to keep track of the best separating axis.
  struct EPAxis {
    internal enum AxisType {
      Unknown,
      EdgeA,
      EdgeB
    };

    internal AxisType type;
    internal int32    index;
    internal float    separation;
  };

// This holds polygon B expressed in frame A.
  class TempPolygon {
    internal b2Vec2[] vertices = new Vector2[Settings.MaxPolygonVertices];
    internal b2Vec2[] normals  = new Vector2[Settings.MaxPolygonVertices];
    internal int32    count;
  };

// Reference face used for clipping
  struct ReferenceFace {
    internal int32 i1;
    internal int32 i2;

    internal b2Vec2 v1;
    internal b2Vec2 v2;

    internal b2Vec2 normal;

    internal b2Vec2 sideNormal1;
    internal float  sideOffset1;

    internal b2Vec2 sideNormal2;
    internal float  sideOffset2;
  };

// This class collides and edge and a polygon, taking into account edge adjacency.
  struct EPCollider {
    // enum VertexType {
    //   e_isolated,
    //   e_concave,
    //   e_convex
    // };

    TempPolygon m_polygonB;

    Transform  m_xf;
    b2Vec2     m_centroidB;
    b2Vec2     m_v0,      m_v1,      m_v2, m_v3;
    b2Vec2     m_normal0, m_normal1, m_normal2;
    b2Vec2     m_normal;
    // VertexType m_type1,      m_type2;
    b2Vec2     m_lowerLimit, m_upperLimit;
    float      m_radius;
    bool       m_front;


// Algorithm:
// 1. Classify v1 and v2
// 2. Classify polygon centroid as front or back
// 3. Flip normal if necessary
// 4. Initialize normal range to [-pi, pi] about face normal
// 5. Adjust normal range according to adjacent edges
// 6. Visit each separating axes, only accept axes within the range
// 7. Return if _any_ axis indicates separation
// 8. Clip
    internal void Collide(out Manifold     manifold, in EdgeShape edgeA, in Transform xfA,
      in                      PolygonShape polygonB, in Transform xfB) {
      m_xf        = Math.MulT(xfA, xfB);
      
      m_centroidB = Math.Mul(m_xf, polygonB.m_centroid);
      
      m_v0        = edgeA.m_vertex0;
      m_v1        = edgeA.m_vertex1;
      m_v2        = edgeA.m_vertex2;
      m_v3        = edgeA.m_vertex3;
      
      bool   hasVertex0 = edgeA.m_hasVertex0;
      bool   hasVertex3 = edgeA.m_hasVertex3;
      
      b2Vec2 edge1      = Vector2.Normalize(m_v2 - m_v1);
      m_normal1 = new Vector2(edge1.Y, -edge1.X);
      float offset1 = Vector2.Dot(m_normal1, m_centroidB - m_v1);
      float offset0 = 0.0f, offset2 = 0.0f;

      bool convex1 = false, convex2 = false;

      // Is there a preceding edge?
      if (hasVertex0) {
        b2Vec2 edge0 = Vector2.Normalize(m_v1 - m_v0);
        m_normal0 = new Vector2(edge0.Y, -edge0.X);
        convex1   = Vectex.Cross(edge0, edge1) >= 0.0f;
        offset0   = Vector2.Dot(m_normal0, m_centroidB - m_v0);
      }

      // Is there a following edge?
      if (hasVertex3) {
        b2Vec2 edge2 = Vector2.Normalize(m_v3 - m_v2);
        m_normal2 = new Vector2(edge2.Y, -edge2.X);
        convex2   = Vectex.Cross(edge1, edge2) > 0.0f;
        offset2   = Vector2.Dot(m_normal2, m_centroidB - m_v2);
      }

      // Determine front or back collision. Determine collision normal limits.
      if (hasVertex0 && hasVertex3) {
        if (convex1 && convex2) {
          m_front = offset0 >= 0.0f || offset1 >= 0.0f || offset2 >= 0.0f;
          if (m_front) {
            m_normal     = m_normal1;
            m_lowerLimit = m_normal0;
            m_upperLimit = m_normal2;
          }
          else {
            m_normal     = -m_normal1;
            m_lowerLimit = -m_normal1;
            m_upperLimit = -m_normal1;
          }
        }
        else if (convex1) {
          m_front = offset0 >= 0.0f || (offset1 >= 0.0f && offset2 >= 0.0f);
          if (m_front) {
            m_normal     = m_normal1;
            m_lowerLimit = m_normal0;
            m_upperLimit = m_normal1;
          }
          else {
            m_normal     = -m_normal1;
            m_lowerLimit = -m_normal2;
            m_upperLimit = -m_normal1;
          }
        }
        else if (convex2) {
          m_front = offset2 >= 0.0f || (offset0 >= 0.0f && offset1 >= 0.0f);
          if (m_front) {
            m_normal     = m_normal1;
            m_lowerLimit = m_normal1;
            m_upperLimit = m_normal2;
          }
          else {
            m_normal     = -m_normal1;
            m_lowerLimit = -m_normal1;
            m_upperLimit = -m_normal0;
          }
        }
        else {
          m_front = offset0 >= 0.0f && offset1 >= 0.0f && offset2 >= 0.0f;
          if (m_front) {
            m_normal     = m_normal1;
            m_lowerLimit = m_normal1;
            m_upperLimit = m_normal1;
          }
          else {
            m_normal     = -m_normal1;
            m_lowerLimit = -m_normal2;
            m_upperLimit = -m_normal0;
          }
        }
      }
      else if (hasVertex0) {
        if (convex1) {
          m_front = offset0 >= 0.0f || offset1 >= 0.0f;
          if (m_front) {
            m_normal     = m_normal1;
            m_lowerLimit = m_normal0;
            m_upperLimit = -m_normal1;
          }
          else {
            m_normal     = -m_normal1;
            m_lowerLimit = m_normal1;
            m_upperLimit = -m_normal1;
          }
        }
        else {
          m_front = offset0 >= 0.0f && offset1 >= 0.0f;
          if (m_front) {
            m_normal     = m_normal1;
            m_lowerLimit = m_normal1;
            m_upperLimit = -m_normal1;
          }
          else {
            m_normal     = -m_normal1;
            m_lowerLimit = m_normal1;
            m_upperLimit = -m_normal0;
          }
        }
      }

      else if (hasVertex3) {
        if (convex2) {
          m_front = offset1 >= 0.0f || offset2 >= 0.0f;
          if (m_front) {
            m_normal     = m_normal1;
            m_lowerLimit = -m_normal1;
            m_upperLimit = m_normal2;
          }
          else {
            m_normal     = -m_normal1;
            m_lowerLimit = -m_normal1;
            m_upperLimit = m_normal1;
          }
        }
        else {
          m_front = offset1 >= 0.0f && offset2 >= 0.0f;
          if (m_front) {
            m_normal     = m_normal1;
            m_lowerLimit = -m_normal1;
            m_upperLimit = m_normal1;
          }
          else {
            m_normal     = -m_normal1;
            m_lowerLimit = -m_normal2;
            m_upperLimit = m_normal1;
          }
        }
      }

      else {
        m_front = offset1 >= 0.0f;
        if (m_front) {
          m_normal     = m_normal1;
          m_lowerLimit = -m_normal1;
          m_upperLimit = -m_normal1;
        }
        else {
          m_normal     = -m_normal1;
          m_lowerLimit = m_normal1;
          m_upperLimit = m_normal1;
        }
      }

      // Get polygonB in frameA
      m_polygonB = new TempPolygon();
      m_polygonB.count = polygonB.m_count;
      for (int32 i = 0; i < polygonB.m_count; ++i) {
        m_polygonB.vertices[i] = Math.Mul(m_xf,   polygonB.m_vertices[i]);
        m_polygonB.normals[i]  = Math.Mul(m_xf.q, polygonB.m_normals[i]);
      }

      m_radius            = polygonB.m_radius + edgeA.m_radius;
      manifold            = new Manifold();
      manifold.pointCount = 0;

      EPAxis edgeAxis = ComputeEdgeSeparation();

      // If no valid normal can be found than this edge should not collide.
      if (edgeAxis.type == EPAxis.AxisType.Unknown) {
        return;
      }

      if (edgeAxis.separation > m_radius) {
        return;
      }

      EPAxis polygonAxis = ComputePolygonSeparation();
      if (polygonAxis.type != EPAxis.AxisType.Unknown && polygonAxis.separation > m_radius) {
        return;
      }

      // Use hysteresis for jitter reduction.
      const float k_relativeTol = 0.98f;
      const float k_absoluteTol = 0.001f;

      EPAxis primaryAxis;
      if (polygonAxis.type == EPAxis.AxisType.Unknown) {
        primaryAxis = edgeAxis;
      }
      else if (polygonAxis.separation > k_relativeTol * edgeAxis.separation + k_absoluteTol) {
        primaryAxis = polygonAxis;
      }
      else {
        primaryAxis = edgeAxis;
      }

      ClipVertex[] ie = new ClipVertex[2];

      ReferenceFace rf;
      if (primaryAxis.type == EPAxis.AxisType.EdgeA) {
        manifold.type = ManifoldType.FaceA;

        // Search for the polygon normal that is most anti-parallel to the edge normal.
        int32 bestIndex = 0;
        float bestValue = Vector2.Dot(m_normal, m_polygonB.normals[0]);
        for (int32 i = 1; i < m_polygonB.count; ++i) {
          float value = Vector2.Dot(m_normal, m_polygonB.normals[i]);
          if (value < bestValue) {
            bestValue = value;
            bestIndex = i;
          }
        }

        int32 i1 = bestIndex;
        int32 i2 = i1 + 1 < m_polygonB.count ? i1 + 1 : 0;

        ie[0].v            = m_polygonB.vertices[i1];
        ie[0].id.cf.indexA = 0;
        ie[0].id.cf.indexB = (byte) (i1);
        ie[0].id.cf.typeA  = (byte) ContactFeatureType.Face;
        ie[0].id.cf.typeB  = (byte) ContactFeatureType.Vertex;

        ie[1].v            = m_polygonB.vertices[i2];
        ie[1].id.cf.indexA = 0;
        ie[1].id.cf.indexB = (byte) (i2);
        ie[1].id.cf.typeA  = (byte) ContactFeatureType.Face;
        ie[1].id.cf.typeB  = (byte) ContactFeatureType.Vertex;

        if (m_front) {
          rf.i1     = 0;
          rf.i2     = 1;
          rf.v1     = m_v1;
          rf.v2     = m_v2;
          rf.normal = m_normal1;
        }
        else {
          rf.i1     = 1;
          rf.i2     = 0;
          rf.v1     = m_v2;
          rf.v2     = m_v1;
          rf.normal = -m_normal1;
        }
      }
      else {
        manifold.type = ManifoldType.FaceB;

        ie[0].v            = m_v1;
        ie[0].id.cf.indexA = 0;
        ie[0].id.cf.indexB = (byte) (primaryAxis.index);
        ie[0].id.cf.typeA  = (byte) ContactFeatureType.Vertex;
        ie[0].id.cf.typeB  = (byte) ContactFeatureType.Face;

        ie[1].v            = m_v2;
        ie[1].id.cf.indexA = 0;
        ie[1].id.cf.indexB = (byte) (primaryAxis.index);
        ie[1].id.cf.typeA  = (byte) ContactFeatureType.Vertex;
        ie[1].id.cf.typeB  = (byte) ContactFeatureType.Face;

        rf.i1     = primaryAxis.index;
        rf.i2     = rf.i1 + 1 < m_polygonB.count ? rf.i1 + 1 : 0;
        rf.v1     = m_polygonB.vertices[rf.i1];
        rf.v2     = m_polygonB.vertices[rf.i2];
        rf.normal = m_polygonB.normals[rf.i1];
      }

      rf.sideNormal1 = new Vector2(rf.normal.Y, -rf.normal.X);
      rf.sideNormal2 = -rf.sideNormal1;
      rf.sideOffset1 = Vector2.Dot(rf.sideNormal1, rf.v1);
      rf.sideOffset2 = Vector2.Dot(rf.sideNormal2, rf.v2);

      // Clip incident edge against extruded edge1 side edges.
      int32        np;

      // Clip to box side 1
      np = Collision.ClipSegmentToLine(out ClipVertex[] clipPoints1, ie, rf.sideNormal1, rf.sideOffset1, rf.i1);
      if (np < Settings.MaxManifoldPoints) {
        return;
      }

      // Clip to negative box side 1
      np = Collision.ClipSegmentToLine(out ClipVertex[] clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2);
      if (np < Settings.MaxManifoldPoints) {
        return;
      }

      // Now clipPoints2 contains the clipped points.
      if (primaryAxis.type == EPAxis.AxisType.EdgeA) {
        manifold.localNormal = rf.normal;
        manifold.localPoint  = rf.v1;
      }
      else {
        manifold.localNormal = polygonB.m_normals[rf.i1];
        manifold.localPoint  = polygonB.m_vertices[rf.i1];
      }

      int32 pointCount = 0;
      for (int32 i = 0; i < Settings.MaxManifoldPoints; ++i) {
        float separation = Vector2.Dot(rf.normal, clipPoints2[i].v - rf.v1);

        if (separation <= m_radius) {
          ManifoldPoint cp = new ManifoldPoint();

          if (primaryAxis.type == EPAxis.AxisType.EdgeA) {
            cp.localPoint = Math.MulT(m_xf, clipPoints2[i].v);
            cp.id         = clipPoints2[i].id;
          }
          else {
            cp.localPoint   = clipPoints2[i].v;
            cp.id.cf.typeA  = clipPoints2[i].id.cf.typeB;
            cp.id.cf.typeB  = clipPoints2[i].id.cf.typeA;
            cp.id.cf.indexA = clipPoints2[i].id.cf.indexB;
            cp.id.cf.indexB = clipPoints2[i].id.cf.indexA;
          }

          manifold.points[pointCount] = cp;
          ++pointCount;
        }
      }

      manifold.pointCount = pointCount;
    }

    EPAxis ComputeEdgeSeparation() {
      EPAxis axis;
      axis.type       = EPAxis.AxisType.EdgeA;
      axis.index      = m_front ? 0 : 1;
      axis.separation = float.MaxValue;
      for (int32 i = 0; i < m_polygonB.count; ++i) {
        float s = Vector2.Dot(m_normal, m_polygonB.vertices[i] - m_v1);
        if (s < axis.separation) {
          axis.separation = s;
        }
      }

      return axis;
    }

    EPAxis ComputePolygonSeparation() {
      EPAxis axis;
      axis.type       = EPAxis.AxisType.Unknown;
      axis.index      = -1;
      axis.separation = float.MinValue;
      b2Vec2 perp = new Vector2(-m_normal.Y, m_normal.X);
      for (int32 i = 0; i < m_polygonB.count; ++i) {
        b2Vec2 n = -m_polygonB.normals[i];

        float s1 = Vector2.Dot(n, m_polygonB.vertices[i] - m_v1);
        float s2 = Vector2.Dot(n, m_polygonB.vertices[i] - m_v2);
        float s  = MathF.Min(s1, s2);

        if (s > m_radius) {
          // No collision
          axis.type       = EPAxis.AxisType.EdgeB;
          axis.index      = i;
          axis.separation = s;
          return axis;
        }

        // Adjacency
        if (Vector2.Dot(n, perp) >= 0.0f) {
          if (Vector2.Dot(n - m_upperLimit, m_normal) < -Settings.AngularSlop) {
            continue;
          }
        }
        else {
          if (Vector2.Dot(n - m_lowerLimit, m_normal) < -Settings.AngularSlop) {
            continue;
          }
        }

        if (s > axis.separation) {
          axis.type       = EPAxis.AxisType.EdgeB;
          axis.index      = i;
          axis.separation = s;
        }
      }

      return axis;
    }
  }
  
  enum ContactFeatureType:byte
  {
    Vertex = 0,
    Face   = 1
  };
  
  /// The features that intersect to form the contact point
  /// This must be 4 bytes or less.
  struct ContactFeature
  {
    internal uint8 indexA;		///< Feature index on shapeA
    internal uint8 indexB;		///< Feature index on shapeB
    internal uint8 typeA;		///< The feature type on shapeA
    internal uint8 typeB; ///< The feature type on shapeB
  };
}