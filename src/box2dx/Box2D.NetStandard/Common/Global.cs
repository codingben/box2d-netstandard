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
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Box2D.NetStandard.Collision;

namespace Box2D.NetStandard.Common
{
    public class Global
    {
        public static void Swap<T>(ref T a, ref T b)
        {
            T c = a;
            a = b;
            b = c;
        }

        public static void GetPointStates(
            out PointState[] state1,
            out PointState[] state2,
            in Manifold manifold1,
            in Manifold manifold2)
        {
            state1 = new PointState[Settings.MaxManifoldPoints];
            state2 = new PointState[Settings.MaxManifoldPoints];
            for (var i = 0; i < Settings.MaxManifoldPoints; ++i)
            {
                state1[i] = PointState.Null;
                state2[i] = PointState.Null;
            }

            // Detect persists and removes.
            for (var i = 0; i < manifold1.pointCount; ++i)
            {
                ContactID id = manifold1.points[i].id;

                state1[i] = PointState.Remove;

                for (var j = 0; j < manifold2.pointCount; ++j)
                {
                    if (manifold2.points[j].id.key == id.key)
                    {
                        state1[i] = PointState.Persist;
                        break;
                    }
                }
            }

            // Detect persists and adds.
            for (var i = 0; i < manifold2.pointCount; ++i)
            {
                ContactID id = manifold2.points[i].id;

                state2[i] = PointState.Add;

                for (var j = 0; j < manifold1.pointCount; ++j)
                {
                    if (manifold1.points[j].id.key == id.key)
                    {
                        state2[i] = PointState.Persist;
                        break;
                    }
                }
            }
        }
    }
}