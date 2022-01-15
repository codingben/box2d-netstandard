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
using Box2D.NetStandard.Common;

namespace Box2D.NetStandard.Collision.Shapes
{
    /// <summary>
    ///  A circle shape.
    /// </summary>
    public class CircleShape : Shape
    {
        internal const byte contactMatch = 0;
        internal Vector2 m_p;

        public CircleShape()
        {
            m_radius = 0;
            m_p = Vector2.Zero;
        }

        public Vector2 Center
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => m_p;
            set => m_p = value;
        }

        public float Radius
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => m_radius;
            set => m_radius = value;
        }

        internal override byte ContactMatch => contactMatch;

        public override Shape Clone() => (CircleShape)MemberwiseClone();

        public override int GetChildCount() => 1;

        public override bool TestPoint(in Transform transform, in Vector2 p)
        {
            Vector2 center = transform.p + Vector2.Transform(m_p, transform.q); //   Math.Mul(transform.q, m_p);
            Vector2 d = p - center;
            return Vector2.Dot(d, d) <= m_radius * m_radius;
        }

        public override bool RayCast(
            out RayCastOutput output,
            in RayCastInput input,
            in Transform transform,
            int childIndex)
        {
            output = default;

            Vector2 position = transform.p + Vector2.Transform(m_p, transform.q); // Math.Mul(transform.q, m_p);
            Vector2 s = input.p1 - position;
            float b = Vector2.Dot(s, s) - m_radius * m_radius;

            // Solve quadratic equation.
            Vector2 r = input.p2 - input.p1;
            float c = Vector2.Dot(s, r);
            float rr = Vector2.Dot(r, r);
            float sigma = c * c - rr * b;

            // Check for negative discriminant and short segment.
            if (sigma < 0.0f || rr < Settings.FLT_EPSILON)
            {
                return false;
            }

            // Find the point of intersection of the line with the circle.
            float a = -(c + MathF.Sqrt(sigma));

            // Is the intersection point on the segment?
            if (0.0f <= a && a <= input.maxFraction * rr)
            {
                a /= rr;
                output.fraction = a;
                output.normal = Vector2.Normalize(s + a * r);
                return true;
            }

            return false;
        }

        public override void ComputeAABB(out AABB aabb, in Transform transform, int childIndex)
        {
            Vector2 p = transform.p + Vector2.Transform(m_p, transform.q); // Math.Mul(transform.q, m_p);
            aabb.lowerBound = new Vector2(p.X - m_radius, p.Y - m_radius);
            aabb.upperBound = new Vector2(p.X + m_radius, p.Y + m_radius);
        }

        public override void ComputeMass(out MassData massData, float density)
        {
            massData.mass = density * Settings.Pi * m_radius * m_radius;
            massData.center = m_p;

            // inertia about the local origin
            massData.I = massData.mass * (0.5f * m_radius * m_radius + Vector2.Dot(m_p, m_p));
        }

        public void Set(in Vector2 center, in float radius)
        {
            m_p = center;
            m_radius = radius;
        }
    }
}