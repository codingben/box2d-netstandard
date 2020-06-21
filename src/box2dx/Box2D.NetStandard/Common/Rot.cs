using System;
using System.Numerics;

namespace Box2D.NetStandard.Common {
  public struct Rot {
    /// Sine and cosine
    internal float s;

    /// Sine and cosine
    internal float c;

    /// Initialize from an angle in radians
    internal Rot(float angle) {
      s = MathF.Sin(angle);
      c = MathF.Cos(angle);
    }

    /// Set using an angle in radians.
    internal void Set(float angle) {
      s = MathF.Sin(angle);
      c = MathF.Cos(angle);
    }

    /// Set to the identity rotation
    void SetIdentity() {
      s = 0.0f;
      c = 1.0f;
    }

    /// Get the angle in radians
    float GetAngle() {
      return MathF.Atan2(s, c);
    }

    /// Get the x-axis
    Vector2 GetXAxis() {
      return new Vector2(c, s);
    }

    /// Get the u-axis
    Vector2 GetYAxis() {
      return new Vector2(-s, c);
    }
  };
}