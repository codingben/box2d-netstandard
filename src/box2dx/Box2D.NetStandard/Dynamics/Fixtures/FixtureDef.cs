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

using Box2D.NetStandard.Collision.Shapes;

namespace Box2D.NetStandard.Dynamics.Fixtures
{
    /// <summary>
    ///  A fixture definition is used to create a fixture. This class defines an
    ///  abstract fixture definition. You can reuse fixture definitions safely.
    /// </summary>
    public class FixtureDef
    {
        /// <summary>
        ///  The density, usually in kg/m^2.
        /// </summary>
        public float density;

        /// <summary>
        ///  Contact filtering data.
        /// </summary>
        public Filter filter = new Filter();

        /// <summary>
        ///  The friction coefficient, usually in the range [0,1].
        /// </summary>
        public float friction = 0.2f;

        /// <summary>
        ///  A sensor shape collects contact information but never generates a collision response.
        /// </summary>
        public bool isSensor;

        /// <summary>
        ///  The restitution (elasticity) usually in the range [0,1].
        /// </summary>
        public float restitution;

        public Shape shape;

        /// <summary>
        ///  Use this to store application specific fixture data.
        /// </summary>
        public object userData;
    }
}