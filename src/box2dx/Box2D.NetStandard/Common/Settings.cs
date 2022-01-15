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

namespace Box2D.NetStandard.Common
{
    public class Settings
    {
        public const int MaxTOIIterations = 20;

        public const float AABBMultiplier = 4.0f;
        public const float AABBExtension = 0.1f;
        public const int MaxTOIContacts = 32;
        public const int MaxSubSteps = 8;

        public const float FLT_EPSILON = float.Epsilon; //1.192092896e-07F;//smallest such that 1.0f+FLT_EPSILON != 1.0f
        public const float FLT_EPSILON_SQUARED = FLT_EPSILON * FLT_EPSILON; //smallest such that 1.0f+FLT_EPSILON != 1.0f

        public const float Pib2 = 3.14159265359f;        // Original code. Comes out at 3.1415927f
        public const float Pi = MathF.PI;                // Also 3.1415927f
        public const float Pi2 = (float)System.Math.PI; // Also displayed as "3.1415927f"
        public const float Tau = 2f * Pi;

        // Global tuning constants based on meters-kilograms-seconds (MKS) units.

        // Collision
        public const int MaxManifoldPoints = 2;
        public const int MaxPolygonVertices = 8;
        public const int MaxProxies = 512;          // this must be a power of two
        public const int MaxPairs = 8 * MaxProxies; // this must be a power of two

        // Dynamics

        /// <summary>
        ///  A small length used as a collision and constraint tolerance. Usually it is
        ///  chosen to be numerically significant, but visually insignificant.
        /// </summary>
        public const float LinearSlop = 0.005f; // 0.5 cm

        /// <summary>
        ///  A small angle used as a collision and constraint tolerance. Usually it is
        ///  chosen to be numerically significant, but visually insignificant.
        /// </summary>
        public const float AngularSlop = 2.0f / 180.0f * Pi; // 2 degrees

        /// <summary>
        ///  The radius of the polygon/edge shape skin. This should not be modified. Making
        ///  this smaller means polygons will have and insufficient for continuous collision.
        ///  Making it larger may create artifacts for vertex collision.
        /// </summary>
        public const float PolygonRadius = 2.0f * LinearSlop;

        /// <summary>
        ///  Continuous collision detection (CCD) works with core, shrunken shapes. This is amount
        ///  by which shapes are automatically shrunk to work with CCD.
        ///  This must be larger than LinearSlop.
        /// </summary>
        public const float ToiSlop = 8.0f * LinearSlop;

        /// <summary>
        ///  Maximum number of contacts to be handled to solve a TOI island.
        /// </summary>
        public const int MaxTOIContactsPerIsland = 32;

        /// <summary>
        ///  Maximum number of joints to be handled to solve a TOI island.
        /// </summary>
        public const int MaxTOIJointsPerIsland = 32;

        /// <summary>
        ///  A velocity threshold for elastic collisions. Any collision with a relative linear
        ///  velocity below this threshold will be treated as inelastic.
        /// </summary>
        public const float VelocityThreshold = 1.0f; // 1 m/s

        /// <summary>
        ///  The maximum linear position correction used when solving constraints.
        ///  This helps to prevent overshoot.
        /// </summary>
        public const float MaxLinearCorrection = 0.2f; // 20 cm

        /// <summary>
        ///  The maximum angular position correction used when solving constraints.
        ///  This helps to prevent overshoot.
        /// </summary>
        public const float MaxAngularCorrection = 8.0f / 180.0f * Pi; // 8 degrees

        /// <summary>
        ///  The maximum linear velocity of a body. This limit is very large and is used
        ///  to prevent numerical problems. You shouldn't need to adjust this.
        /// </summary>
        public const float MaxLinearVelocity = 200.0f;

        public const float MaxLinearVelocitySquared = MaxLinearVelocity * MaxLinearVelocity;

        /// <summary>
        ///  The maximum angular velocity of a body. This limit is very large and is used
        ///  to prevent numerical problems. You shouldn't need to adjust this.
        /// </summary>
        public const float MaxAngularVelocity = 250.0f;

        /// <summary>
        ///  The maximum linear velocity of a body. This limit is very large and is used
        ///  to prevent numerical problems. You shouldn't need to adjust this.
        /// </summary>
        public const float MaxTranslation = 2.0f;

        public const float MaxTranslationSquared = MaxTranslation * MaxTranslation;

        /// <summary>
        ///  The maximum angular velocity of a body. This limit is very large and is used
        ///  to prevent numerical problems. You shouldn't need to adjust this.
        /// </summary>
        public const float MaxRotation = 0.5f * Pi;

        public const float MaxRotationSquared = MaxRotation * MaxRotation;

        /// <summary>
        ///  This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
        ///  that overlap is removed in one time step. However using values close to 1 often lead to overshoot.
        /// </summary>
        public const float ContactBaumgarte = 0.2f;

        // Sleep

        /// <summary>
        ///  The time that a body must be still before it will go to sleep.
        /// </summary>
        public const float TimeToSleep = 0.5f; // half a second

        /// <summary>
        ///  A body cannot sleep if its linear velocity is above this tolerance.
        /// </summary>
        public const float LinearSleepTolerance = 0.01f; // 1 cm/s

        /// <summary>
        ///  A body cannot sleep if its angular velocity is above this tolerance.
        /// </summary>
        public const float AngularSleepTolerance = 2.0f / 180.0f; // 2 degrees/s

        public const bool BlockSolve = true;
        public const float Baumgarte = 0.2f;
        public const float TOIBaumgarte = 0.75f;

        public static float FORCE_SCALE(float x) => x;

        public static float FORCE_INV_SCALE(float x) => x;

        /// <summary>
        ///  Friction mixing law. Feel free to customize this.
        /// </summary>
        public static float MixFriction(float friction1, float friction2) =>
            (float)System.Math.Sqrt(friction1 * friction2);

        /// <summary>
        ///  Restitution mixing law. Feel free to customize this.
        /// </summary>
        public static float MixRestitution(float restitution1, float restitution2) =>
            restitution1 > restitution2 ? restitution1 : restitution2;
    }
}