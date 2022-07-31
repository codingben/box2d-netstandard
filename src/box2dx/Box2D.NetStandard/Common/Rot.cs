using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Box2D.NetStandard.Common
{
    public struct Rot
    {
        /// Sine and cosine
        internal float s;

        /// Sine and cosine
        internal float c;

        /// Initialize from an angle in radians
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal Rot(float angle)
        {
            s = MathF.Sin(angle);
            c = MathF.Cos(angle);
        }

        /// Set using an angle in radians.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void Set(float angle)
        {
            s = MathF.Sin(angle);
            c = MathF.Cos(angle);
        }

        /// Set to the identity rotation
        private void SetIdentity()
        {
            s = 0.0f;
            c = 1.0f;
        }

        /// Get the angle in radians
        private float GetAngle() => MathF.Atan2(s, c);

        /// Get the x-axis
        private Vector2 GetXAxis() => new Vector2(c, s);

        /// Get the u-axis
        private Vector2 GetYAxis() => new Vector2(-s, c);
    }
}