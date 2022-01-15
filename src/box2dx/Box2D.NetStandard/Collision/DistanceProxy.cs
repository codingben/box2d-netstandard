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
using Box2D.NetStandard.Collision.Shapes;

namespace Box2D.NetStandard.Collision
{
    public struct DistanceProxy
    {
        internal Vector2[] _buffer; // = new Vector2[2];
        internal Vector2[] _vertices;
        internal int _count;
        internal float _radius;

        private void Set(in Vector2[] vertices, int count, float radius)
        {
            _vertices = vertices;
            _count = count;
            _radius = radius;
        }

        internal void Set(in Shape shape, in int index)
        {
            switch (shape)
            {
                case CircleShape circle:
                    _vertices = new[] { circle.m_p };
                    _count = 1;
                    _radius = circle.m_radius;
                    break;
                case PolygonShape polygon:
                    _vertices = polygon.m_vertices;
                    _count = polygon.m_count;
                    _radius = polygon.m_radius;
                    break;
                case ChainShape chain:
                    _buffer = new Vector2[2];
                    _buffer[0] = chain.m_vertices[index];
                    if (index + 1 < chain.m_count)
                    {
                        _buffer[1] = chain.m_vertices[index + 1];
                    }
                    else
                    {
                        _buffer[1] = chain.m_vertices[0];
                    }

                    _vertices = _buffer;
                    _count = 2;
                    _radius = chain.m_radius;

                    break;
                case EdgeShape edge:
                    _vertices = new[] { edge.m_vertex1, edge.m_vertex2 };
                    _count = 2;
                    _radius = edge.m_radius;
                    break;

                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        public int GetSupport(Vector2 d)
        {
            var bestIndex = 0;
            float bestValue = Vector2.Dot(_vertices[0], d);

            for (var i = 1; i < _count; ++i)
            {
                float value = Vector2.Dot(_vertices[i], d);
                if (value > bestValue)
                {
                    bestIndex = i;
                    bestValue = value;
                }
            }

            return bestIndex;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetVertexCount() => _count;

        public int VertexCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => _count;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2 GetVertex(int index) => _vertices[index];

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetSupport(in Vector2 d)
        {
            var bestIndex = 0;
            float bestValue = Vector2.Dot(_vertices[0], d);

            for (var i = 1; i < _count; ++i)
            {
                float value = Vector2.Dot(_vertices[i], d);
                if (value > bestValue)
                {
                    bestIndex = i;
                    bestValue = value;
                }
            }

            return bestIndex;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector2 GetSupportVertex(in Vector2 d)
        {
            var bestIndex = 0;
            float bestValue = Vector2.Dot(_vertices[0], d);

            for (var i = 1; i < _count; ++i)
            {
                float value = Vector2.Dot(_vertices[i], d);
                if (value > bestValue)
                {
                    bestIndex = i;
                    bestValue = value;
                }
            }

            return _vertices[bestIndex];
        }
    }
}