using System;
using System.Numerics;
using Box2D.NetStandard.Common;
using b2Vec2 = System.Numerics.Vector2;
using int32 = System.Int32;

namespace Box2D.NetStandard.Collision {
  /// <summary>
  /// An axis aligned bounding box.
  /// </summary>
  public struct AABB {

    /// <summary>
    /// The lower vertex
    /// </summary>
    internal b2Vec2 lowerBound;

    /// <summary>
    /// The upper vertex
    /// </summary>
    internal b2Vec2 upperBound;

    /// Get the center of the AABB.
    internal b2Vec2 GetCenter() {
      return 0.5f * (lowerBound + upperBound);
    }

    /// Get the extents of the AABB (half-widths).
    internal b2Vec2 GetExtents() {
      return 0.5f * (upperBound - lowerBound);
    }

    /// Get the perimeter length
    internal float GetPerimeter() {
      float wx = upperBound.X - lowerBound.X;
      float wy = upperBound.Y - lowerBound.Y;
      return 2.0f * (wx + wy);
    }

    /// Combine an AABB into this one.
    void Combine(in AABB aabb) {
      lowerBound = Vector2.Min(lowerBound, aabb.lowerBound);
      upperBound = Vector2.Max(upperBound, aabb.upperBound);
    }

    /// Combine two AABBs into this one.
    internal void Combine(in AABB aabb1, in AABB aabb2) {
      lowerBound = Vector2.Min(aabb1.lowerBound, aabb2.lowerBound);
      upperBound = Vector2.Max(aabb1.upperBound, aabb2.upperBound);
    }

    /// Does this aabb contain the provided AABB.
    internal bool Contains(in AABB aabb) {
      bool result = true;
      result = result && lowerBound.X      <= aabb.lowerBound.X;
      result = result && lowerBound.Y      <= aabb.lowerBound.Y;
      result = result && aabb.upperBound.X <= upperBound.X;
      result = result && aabb.upperBound.Y <= upperBound.Y;
      return result;
    }

    bool RayCast(out RayCastOutput output, in RayCastInput input) {
      output = default;
      float tmin = float.MinValue;
      float tmax = float.MaxValue;

      b2Vec2 p    = input.p1;
      b2Vec2 d    = input.p2 - input.p1;
      b2Vec2 absD = Vector2.Abs(d);

      b2Vec2 normal = Vector2.Zero;

      for (int32 i = 0; i < 2; ++i) {
        if (absD.GetIdx(i) < Settings.FLT_EPSILON) {
          // Parallel.
          if (p.GetIdx(i) < lowerBound.GetIdx(i) || upperBound.GetIdx(i) < p.GetIdx(i)) {
            return false;
          }
        }
        else {
          float inv_d = 1.0f                                 / d.GetIdx(i);
          float t1    = (lowerBound.GetIdx(i) - p.GetIdx(i)) * inv_d;
          float t2    = (upperBound.GetIdx(i) - p.GetIdx(i)) * inv_d;

          // Sign of the normal vector.
          float s = -1.0f;

          if (t1 > t2) {
            float temp = t1;
            t1 = t2;
            t2 = temp;
            s = 1.0f;
          }

          // Push the min up
          if (t1 > tmin) {
            normal = new Vector2(i == 0 ? s : 0, i == 1 ? s : 0);
            tmin   = t1;
          }

          // Pull the max down
          tmax = MathF.Min(tmax, t2);

          if (tmin > tmax) {
            return false;
          }
        }
      }

      // Does the ray start inside the box?
      // Does the ray intersect beyond the max fraction?
      if (tmin < 0.0f || input.maxFraction < tmin) {
        return false;
      }

      // Intersection.
      output.fraction = tmin;
      output.normal   = normal;
      return true;
    }

    bool IsValid() {
      b2Vec2 d     = upperBound - lowerBound;
      bool   valid = d.X >= 0.0f && d.Y >= 0.0f;
      valid = valid && lowerBound.IsValid() && upperBound.IsValid();
      return valid;
    }

    public AABB(Vector2 lowerBound, Vector2 upperBound) {
      this.lowerBound = lowerBound;
      this.upperBound = upperBound;
    }
  }
}