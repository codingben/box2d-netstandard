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

using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;

namespace Box2D.NetStandard.Collision
{
    internal class Simplex
    {
        internal readonly SimplexVertex[] m_v;

        internal int m_count;

        public Simplex() => m_v = new SimplexVertex[3];

        private SimplexVertex m_v1
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => m_v[0];
        }

        private SimplexVertex m_v2
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => m_v[1];
        }

        private SimplexVertex m_v3
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => m_v[2];
        }

        internal void ReadCache(
            in SimplexCache cache,
            in DistanceProxy proxyA,
            in Transform transformA,
            in DistanceProxy proxyB,
            in Transform transformB)
        {
            //Debug.Assert(cache.count <= 3);

            // Copy data from cache.
            m_count = cache.count;
            SimplexVertex[] vertices = m_v;
            for (var i = 0; i < m_count; ++i)
            {
                ref SimplexVertex v = ref vertices[i];
                v.indexA = cache.indexA[i];
                v.indexB = cache.indexB[i];
                Vector2 wALocal = proxyA.GetVertex(v.indexA);
                Vector2 wBLocal = proxyB.GetVertex(v.indexB);
                v.wA = Math.Mul(transformA, wALocal);
                v.wB = Math.Mul(transformB, wBLocal);
                v.w = v.wB - v.wA;
                v.a = 0.0f;
            }

            // Compute the new simplex metric, if it is substantially different than
            // old metric then flush the simplex.
            if (m_count > 1)
            {
                float metric1 = cache.metric;
                float metric2 = GetMetric();
                if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < Settings.FLT_EPSILON)
                // Reset the simplex.
                {
                    m_count = 0;
                }
            }

            // If the cache is empty or invalid ...
            if (m_count == 0)
            {
                ref SimplexVertex v = ref vertices[0];
                v.indexA = 0;
                v.indexB = 0;
                Vector2 wALocal = proxyA.GetVertex(0);
                Vector2 wBLocal = proxyB.GetVertex(0);
                v.wA = Math.Mul(transformA, wALocal);
                v.wB = Math.Mul(transformB, wBLocal);
                v.w = v.wB - v.wA;
                v.a = 1.0f;
                m_count = 1;
            }
        }

        internal void WriteCache(SimplexCache cache)
        {
            cache.metric = GetMetric();
            cache.count = (ushort)m_count;
            SimplexVertex[] vertices = m_v;
            for (var i = 0; i < m_count; ++i)
            {
                cache.indexA[i] = (byte)vertices[i].indexA;
                cache.indexB[i] = (byte)vertices[i].indexB;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal Vector2 GetSearchDirection()
        {
            switch (m_count)
            {
                case 1:
                    return -m_v1.w;

                case 2:
                    {
                        Vector2 e12 = m_v2.w - m_v1.w;
                        float sgn = Vectex.Cross(e12, -m_v1.w);

                        return sgn > 0.0f ? Vectex.Cross(1.0f, e12) : Vectex.Cross(e12, 1.0f);
                    }

                default:
                    //Debug.Assert(false);
                    return Vector2.Zero;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal Vector2 GetClosestPoint()
        {
            switch (m_count)
            {
                case 0:
                    //Debug.Assert(false);
                    return Vector2.Zero;
                case 1:
                    return m_v1.w;
                case 2:
                    return m_v1.a * m_v1.w + m_v2.a * m_v2.w;
                case 3:
                    return Vector2.Zero;
                default:
                    //Debug.Assert(false);
                    return Vector2.Zero;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void GetWitnessPoints(out Vector2 pA, out Vector2 pB)
        {
            switch (m_count)
            {
                case 1:
                    pA = m_v1.wA;
                    pB = m_v1.wB;
                    break;

                case 2:
                    pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA;
                    pB = m_v1.a * m_v1.wB + m_v2.a * m_v2.wB;
                    break;

                case 3:
                    pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA + m_v3.a * m_v3.wA;
                    pB = pA;
                    break;
                case 0:
                default:
                    pA = default;
                    pB = default;
                    //Debug.Assert(false);
                    break;
            }
        }

        private float GetMetric()
        {
            switch (m_count)
            {
                case 1:
                    return 0.0f;

                case 2:
                    return Vector2.Distance(m_v1.w, m_v2.w);

                case 3:
                    return Vectex.Cross(m_v2.w - m_v1.w, m_v3.w - m_v1.w);
                case 0:
                default:
                    //Debug.Assert(false);
                    return 0.0f;
            }
        }

        // Solve a line segment using barycentric coordinates.
        //
        // p = a1 * w1 + a2 * w2
        // a1 + a2 = 1
        //
        // The vector from the origin to the closest point on the line is
        // perpendicular to the line.
        // e12 = w2 - w1
        // dot(p, e) = 0
        // a1 * dot(w1, e) + a2 * dot(w2, e) = 0
        //
        // 2-by-2 linear system
        // [1      1     ][a1] = [1]
        // [w1.e12 w2.e12][a2] = [0]
        //
        // Define
        // d12_1 =  dot(w2, e12)
        // d12_2 = -dot(w1, e12)
        // d12 = d12_1 + d12_2
        //
        // Solution
        // a1 = d12_1 / d12
        // a2 = d12_2 / d12
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void Solve2()
        {
            Vector2 w1 = m_v1.w;
            Vector2 w2 = m_v2.w;
            Vector2 e12 = w2 - w1;

            // w1 region
            float d12_2 = -Vector2.Dot(w1, e12);
            if (d12_2 <= 0.0f)
            {
                // a2 <= 0, so we clamp it to 0
                m_v[0].a = 1.0f;
                m_count = 1;
                return;
            }

            // w2 region
            float d12_1 = Vector2.Dot(w2, e12);
            if (d12_1 <= 0.0f)
            {
                // a1 <= 0, so we clamp it to 0
                m_v[1].a = 1.0f;
                m_count = 1;
                m_v[0] = m_v[1];
                return;
            }

            // Must be in e12 region.
            float inv_d12 = 1.0f / (d12_1 + d12_2);
            m_v[0].a = d12_1 * inv_d12;
            m_v[1].a = d12_2 * inv_d12;
            m_count = 2;
        }

        // Possible regions:
        // - points[2]
        // - edge points[0]-points[2]
        // - edge points[1]-points[2]
        // - inside the triangle
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void Solve3()
        {
            Vector2 w1 = m_v1.w;
            Vector2 w2 = m_v2.w;
            Vector2 w3 = m_v3.w;

            // Edge12
            // [1      1     ][a1] = [1]
            // [w1.e12 w2.e12][a2] = [0]
            // a3 = 0
            Vector2 e12 = w2 - w1;
            float w1e12 = Vector2.Dot(w1, e12);
            float w2e12 = Vector2.Dot(w2, e12);
            float d12_1 = w2e12;
            float d12_2 = -w1e12;

            // Edge13
            // [1      1     ][a1] = [1]
            // [w1.e13 w3.e13][a3] = [0]
            // a2 = 0
            Vector2 e13 = w3 - w1;
            float w1e13 = Vector2.Dot(w1, e13);
            float w3e13 = Vector2.Dot(w3, e13);
            float d13_1 = w3e13;
            float d13_2 = -w1e13;

            // Edge23
            // [1      1     ][a2] = [1]
            // [w2.e23 w3.e23][a3] = [0]
            // a1 = 0
            Vector2 e23 = w3 - w2;
            float w2e23 = Vector2.Dot(w2, e23);
            float w3e23 = Vector2.Dot(w3, e23);
            float d23_1 = w3e23;
            float d23_2 = -w2e23;

            // Triangle123
            float n123 = Vectex.Cross(e12, e13);

            float d123_1 = n123 * Vectex.Cross(w2, w3);
            float d123_2 = n123 * Vectex.Cross(w3, w1);
            float d123_3 = n123 * Vectex.Cross(w1, w2);

            // w1 region
            if (d12_2 <= 0.0f && d13_2 <= 0.0f)
            {
                m_v[0].a = 1.0f;
                m_count = 1;
                return;
            }

            // e12
            if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
            {
                float inv_d12 = 1.0f / (d12_1 + d12_2);
                m_v[0].a = d12_1 * inv_d12;
                m_v[1].a = d12_1 * inv_d12;
                m_count = 2;
                return;
            }

            // e13
            if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
            {
                float inv_d13 = 1.0f / (d13_1 + d13_2);
                m_v[0].a = d13_1 * inv_d13;
                m_v[2].a = d13_2 * inv_d13;
                m_count = 2;
                m_v[1] = m_v[2];
                return;
            }

            // w2 region
            if (d12_1 <= 0.0f && d23_2 <= 0.0f)
            {
                m_v[1].a = 1.0f;
                m_count = 1;
                m_v[0] = m_v[1];
                return;
            }

            // w3 region
            if (d13_1 <= 0.0f && d23_1 <= 0.0f)
            {
                m_v[2].a = 1.0f;
                m_count = 1;
                m_v[0] = m_v[2];
                return;
            }

            // e23
            if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
            {
                float inv_d23 = 1.0f / (d23_1 + d23_2);
                m_v[1].a = d23_1 * inv_d23;
                m_v[2].a = d23_2 * inv_d23;
                m_count = 2;
                m_v[0] = m_v[2];
                return;
            }

            // Must be in triangle123
            float inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
            m_v[0].a = d123_1 * inv_d123;
            m_v[1].a = d123_2 * inv_d123;
            m_v[2].a = d123_3 * inv_d123;
            m_count = 3;
        }
    }
}