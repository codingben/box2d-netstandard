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

namespace Box2D.NetStandard.Common
{
    /// <summary>
    ///  A transform contains translation and rotation.
    ///  It is used to represent the position and orientation of rigid frames.
    /// </summary>
    public struct Transform
    {
        /// <summary>
        ///  Position
        /// </summary>
        public Vector2 p;

        /// <summary>
        ///  Rotation
        /// </summary>
        // public Mat22 q;
        public Matrix3x2 q;

        /// <summary>
        ///  Initialize using a position vector and a rotation matrix.
        /// </summary>
        /// <param name="position"></param>
        /// <param name="R"></param>
        // public Transform(Vector2 position, Mat22 rotation)
        // {
        // 	p = position;
        // 	q = rotation;
        // }
        public Transform(Vector2 position, Matrix3x2 rotation)
        {
            p = position;
            q = rotation;
        }

        /// <summary>
        ///  Set this to the identity transform.
        /// </summary>
        public void SetIdentity()
        {
            p = Vector2.Zero;
            // q.SetIdentity();
            q = Matrix3x2.Identity;
        }

        /// Set this based on the position and angle.
        public void Set(Vector2 p, float angle)
        {
            this.p = p;
            q = Matrex.CreateRotation(angle); // Actually about twice as fast to use our own function
        }

        /// Calculate the angle that the rotation matrix represents.
        public float GetAngle() =>
            //	|  ex  |  ey  |
            //  +------+------+
            //	| ex.X | ey.X |
            //  | M11  | M12  |
            //  +------+------+
            //  | ex.Y | ey.Y |
            //  | M21  | M22  |
            //  +------+------+
            MathF.Atan2(q.M21, q.M11);

        public static Transform Identity
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => new Transform(Vector2.Zero, Matrix3x2.Identity);
        }
    }
}