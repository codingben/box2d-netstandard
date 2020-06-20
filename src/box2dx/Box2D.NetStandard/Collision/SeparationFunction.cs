using System;
using System.Diagnostics;
using System.Numerics;
using Box2D.NetStandard.Common;
using Math = Box2D.NetStandard.Common.Math;

namespace Box2D.NetStandard.Collision {
  internal struct SeparationFunction {
    private DistanceProxy          m_proxyA;
    private DistanceProxy          m_proxyB;
    private Sweep                  m_sweepA;
    private Sweep                  m_sweepB;
    private SeparationFunctionType m_type;
    private Vector2                m_axis;
    private Vector2                m_localPoint;

    internal enum SeparationFunctionType {
      Points,
      FaceA,
      FaceB
    };

    internal float Initialize(SimplexCache cache,
      in DistanceProxy                     proxyA, in Sweep sweepA,
      in DistanceProxy                     proxyB, in Sweep sweepB,
      float                                t1) {
      unsafe {
        m_proxyA = proxyA;
        m_proxyB = proxyB;
        Int32 count = cache.count;
        Debug.Assert(0 < count && count < 3);

        m_sweepA = sweepA;
        m_sweepB = sweepB;

        m_sweepA.GetTransform(out Transform xfA, t1);
        m_sweepB.GetTransform(out Transform xfB, t1);

        if (count == 1) {
          unsafe {
            m_type = SeparationFunctionType.Points;
            Vector2 localPointA = m_proxyA._vertices[cache.indexA[0]];
            Vector2 localPointB = m_proxyB._vertices[cache.indexB[0]];
            Vector2 pointA      = Common.Math.Mul(xfA, localPointA);
            Vector2 pointB      = Common.Math.Mul(xfB, localPointB);
            m_axis = pointB - pointA;
            float s = m_axis.Length();
            m_axis = Vector2.Normalize(m_axis);
            return s;
          }
        }
        else if (cache.indexA[0] == cache.indexA[1]) {
          // Two points on B and one on A.
          m_type = SeparationFunctionType.FaceB;
          Vector2 localPointB1 = proxyB._vertices[cache.indexB[0]];
          Vector2 localPointB2 = proxyB._vertices[cache.indexB[1]];

          m_axis = Vector2.Normalize(Vectex.Cross(localPointB2 - localPointB1, 1.0f));
          Vector2 normal = Math.Mul(xfB.q, m_axis);

          m_localPoint = 0.5f * (localPointB1 + localPointB2);
          Vector2 pointB = Math.Mul(xfB, m_localPoint);

          Vector2 localPointA = proxyA._vertices[cache.indexA[0]];
          Vector2 pointA      = Math.Mul(xfA, localPointA);

          float s = Vector2.Dot(pointA - pointB, normal);
          if (s < 0.0f) {
            m_axis = -m_axis;
            s      = -s;
          }

          return s;
        }
        else {
          // Two points on A and one or two points on B.
          m_type = SeparationFunctionType.FaceA;
          Vector2 localPointA1 = m_proxyA._vertices[cache.indexA[0]];
          Vector2 localPointA2 = m_proxyA._vertices[cache.indexA[1]];

          m_axis = Vector2.Normalize(Vectex.Cross(localPointA2 - localPointA1, 1.0f));
          Vector2 normal = Math.Mul(xfA.q, m_axis);

          m_localPoint = 0.5f * (localPointA1 + localPointA2);
          Vector2 pointA = Math.Mul(xfA, m_localPoint);

          Vector2 localPointB = m_proxyB._vertices[cache.indexB[0]];
          Vector2 pointB      = Math.Mul(xfB, localPointB);

          float s = Vector2.Dot(pointB - pointA, normal);
          if (s < 0.0f) {
            m_axis = -m_axis;
            s      = -s;
          }

          return s;
        }
      }
    }

    internal float Evaluate(int indexA, int indexB, float t) {
      m_sweepA.GetTransform(out Transform xfA, t);
      m_sweepB.GetTransform(out Transform xfB, t);

      switch (m_type) {
        case SeparationFunctionType.Points: {
          Vector2 localPointA = m_proxyA._vertices[indexA];
          Vector2 localPointB = m_proxyB._vertices[indexB];

          Vector2 pointA     = Math.Mul(xfA, localPointA);
          Vector2 pointB     = Math.Mul(xfB, localPointB);
          float  separation = Vector2.Dot(pointB - pointA, m_axis);

          return separation;
        }

        case SeparationFunctionType.FaceA: {
          Vector2 normal = Math.Mul(xfA.q, m_axis);
          Vector2 pointA = Math.Mul(xfA,   m_localPoint);

          Vector2 localPointB = m_proxyB._vertices[indexB];
          Vector2 pointB      = Math.Mul(xfB, localPointB);

          float separation = Vector2.Dot(pointB - pointA, normal);
          return separation;
        }

        case SeparationFunctionType.FaceB: {
          Vector2 normal = Math.Mul(xfB.q, m_axis);
          Vector2 pointB = Math.Mul(xfB,   m_localPoint);

          Vector2 localPointA = m_proxyA._vertices[indexA];
          Vector2 pointA      = Math.Mul(xfA, localPointA);

          float separation = Vector2.Dot(pointA - pointB, normal);
          return separation;
        }

        default:
          Debug.Assert(false);
          return 0.0f;
      }
    }

    internal float FindMinSeparation(out int indexA, out int indexB, float t) {
      m_sweepA.GetTransform(out Transform xfA, t);
      m_sweepB.GetTransform(out Transform xfB, t);

      switch (m_type) {
        case SeparationFunctionType.Points: {
          Vector2 axisA = Math.MulT(xfA.q, m_axis);
          Vector2 axisB = Math.MulT(xfB.q, -m_axis);

          indexA = m_proxyA.GetSupport(axisA);
          indexB = m_proxyB.GetSupport(axisB);

          Vector2 localPointA = m_proxyA.GetVertex(indexA);
          Vector2 localPointB = m_proxyB.GetVertex(indexB);

          Vector2 pointA = Math.Mul(xfA, localPointA);
          Vector2 pointB = Math.Mul(xfB, localPointB);

          float separation = Vector2.Dot(pointB - pointA, m_axis);
          return separation;
        }

        case SeparationFunctionType.FaceA: {
          Vector2 normal = Math.Mul(xfA.q, m_axis);
          Vector2 pointA = Math.Mul(xfA,   m_localPoint);

          Vector2 axisB = Math.MulT(xfB.q, -normal);

          indexA = -1;
          indexB = m_proxyB.GetSupport(axisB);

          Vector2 localPointB = m_proxyB.GetVertex(indexB);
          Vector2 pointB      = Math.Mul(xfB, localPointB);

          float separation = Vector2.Dot(pointB - pointA, normal);
          return separation;
        }

        case SeparationFunctionType.FaceB: {
          Vector2 normal = Math.Mul(xfB.q, m_axis);
          Vector2 pointB = Math.Mul(xfB,   m_localPoint);

          Vector2 axisA = Math.MulT(xfA.q, -normal);

          indexB = -1;
          indexA = m_proxyA.GetSupport(axisA);

          Vector2 localPointA = m_proxyA.GetVertex(indexA);
          Vector2 pointA      = Math.Mul(xfA, localPointA);

          float separation = Vector2.Dot(pointA - pointB, normal);
          return separation;
        }

        default:
          Debug.Assert(false);
          indexA = -1;
          indexB = -1;
          return 0.0f;
      }
    }
  }
}