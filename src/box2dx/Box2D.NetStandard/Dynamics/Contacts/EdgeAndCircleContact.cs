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
using Box2D.NetStandard.Collision;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Fixtures;

namespace Box2D.NetStandard.Dynamics.Contacts
{
    internal class EdgeAndCircleContact : Contact
    {
        private readonly CircleShape circleB;

        protected EdgeShape edgeA;

        internal EdgeAndCircleContact(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB)
            : base(fixtureA, indexA, fixtureB, indexB)
        {
            //Debug.Assert(fixtureA.Type == ShapeType.Edge);
            //Debug.Assert(fixtureB.Type == ShapeType.Circle);
            m_manifold.pointCount = 0;
            m_manifold.points[0] = new ManifoldPoint();
            m_manifold.points[0].normalImpulse = 0.0f;
            m_manifold.points[0].tangentImpulse = 0.0f;

            edgeA = m_fixtureA.Shape is EdgeShape ? (EdgeShape)m_fixtureA.Shape : null;
            circleB = (CircleShape)m_fixtureB.Shape;
        }

        internal override void Evaluate(out Manifold manifold, in Transform xfA, in Transform xfB)
        {
            manifold = new Manifold();

            //manifold.pointCount = 0;

            Vector2 Q = Math.MulT(xfA, Math.Mul(xfB, circleB.m_p));

            Vector2 A = edgeA.m_vertex1, B = edgeA.m_vertex2;
            Vector2 e = B - A;

            float u = Vector2.Dot(e, B - Q);
            float v = Vector2.Dot(e, Q - A);

            float radius = edgeA.m_radius + circleB.m_radius;

            ContactFeature cf;
            cf.indexB = 0;
            cf.typeB = (byte)ContactFeatureType.Vertex;


            // Region A
            if (v <= 0.0f)
            {
                Vector2 P = A;
                Vector2 d = Q - P;
                float dd = Vector2.Dot(d, d);
                if (dd > radius * radius)
                {
                    return;
                }

                // Is there an edge connected to A?
                if (edgeA.m_vertex0.HasValue)
                {
                    Vector2 A1 = edgeA.m_vertex0.Value;
                    Vector2 B1 = A;
                    Vector2 e1 = B1 - A1;
                    float u1 = Vector2.Dot(e1, B1 - Q);

                    // Is the circle in Region AB of the previous edge?
                    if (u1 > 0.0f)
                    {
                        return;
                    }
                }

                cf.indexA = 0;
                cf.typeA = (byte)ContactFeatureType.Vertex;
                manifold.pointCount = 1;
                manifold.type = ManifoldType.Circles;
                manifold.localNormal = Vector2.Zero;
                manifold.localPoint = P;
                manifold.points[0] = new ManifoldPoint();
                manifold.points[0].id.key = 0;
                manifold.points[0].id.cf = cf;
                manifold.points[0].localPoint = circleB.m_p;
                return;
            }

            // Region B
            if (u <= 0.0f)
            {
                Vector2 P = B;
                Vector2 d = Q - P;
                float dd = Vector2.Dot(d, d);
                if (dd > radius * radius)
                {
                    return;
                }

                // Is there an edge connected to B?
                if (edgeA.m_vertex3.HasValue)
                {
                    Vector2 B2 = edgeA.m_vertex3.Value;
                    Vector2 A2 = B;
                    Vector2 e2 = B2 - A2;
                    float v2 = Vector2.Dot(e2, Q - A2);

                    // Is the circle in Region AB of the next edge?
                    if (v2 > 0.0f)
                    {
                        return;
                    }
                }

                cf.indexA = 1;
                cf.typeA = (byte)ContactFeatureType.Vertex;
                manifold.pointCount = 1;
                manifold.type = ManifoldType.Circles;
                manifold.localNormal = Vector2.Zero;
                manifold.localPoint = P;
                manifold.points[0] = new ManifoldPoint();
                manifold.points[0].id.key = 0;
                manifold.points[0].id.cf = cf;
                manifold.points[0].localPoint = circleB.m_p;
                return;
            }

            {
                // Region AB
                float den = Vector2.Dot(e, e);
                //Debug.Assert(den > 0.0f);
                Vector2 P = 1.0f / den * (u * A + v * B);
                Vector2 d = Q - P;
                float dd = Vector2.Dot(d, d);
                if (dd > radius * radius)
                {
                    return;
                }

                var n = new Vector2(-e.Y, e.X);
                if (Vector2.Dot(n, Q - A) < 0.0f)
                {
                    n = new Vector2(-n.X, -n.Y);
                }

                n = Vector2.Normalize(n);

                cf.indexA = 0;
                cf.typeA = (byte)ContactFeatureType.Face;
                manifold.pointCount = 1;
                manifold.type = ManifoldType.FaceA;
                manifold.localNormal = n;
                manifold.localPoint = A;
                manifold.points[0] = new ManifoldPoint();
                manifold.points[0].id.key = 0;
                manifold.points[0].id.cf = cf;
                manifold.points[0].localPoint = circleB.m_p;
            }
        }
    }
}