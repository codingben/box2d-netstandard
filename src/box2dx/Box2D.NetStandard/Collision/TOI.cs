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
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Contacts;

namespace Box2D.NetStandard.Collision
{
    public static class TOI
    {
        internal static void TimeOfImpact(out TOIOutput output, in TOIInput input)
        {
            //++_toiCalls;
            output.t = input.tMax;
            output.state = TOIOutputState.Unknown;

            DistanceProxy proxyA = input.proxyA;
            DistanceProxy proxyB = input.proxyB;

            Sweep sweepA = input.sweepA;
            Sweep sweepB = input.sweepB;

            // Large rotations can make the root finder fail, so we normalize the
            // sweep angles.
            sweepA.Normalize();
            sweepB.Normalize();

            float tMax = input.tMax;

            float totalRadius = proxyA._radius + proxyB._radius;
            float target = MathF.Max(Settings.LinearSlop, totalRadius - 3.0f * Settings.LinearSlop);
            float tolerance = 0.25f * Settings.LinearSlop;
            //Debug.Assert(target > tolerance);

            var t1 = 0.0f;
            var iter = 0;

            // Prepare input for distance query.
            var cache = new SimplexCache();
            DistanceInput distanceInput;
            distanceInput.proxyA = input.proxyA;
            distanceInput.proxyB = input.proxyB;
            distanceInput.useRadii = false;

            // The outer loop progressively attempts to compute new separating axes.
            // This loop terminates when an axis is repeated (no progress is made).
            SeparationFunction fcn = default;
            for (; ; )
            {
                sweepA.GetTransform(out Transform xfA, t1);
                sweepB.GetTransform(out Transform xfB, t1);

                // Get the distance between shapes. We can also use the results
                // to get a separating axis.
                distanceInput.transformA = xfA;
                distanceInput.transformB = xfB;
                Contact.Distance(out DistanceOutput distanceOutput, cache, in distanceInput);

                // If the shapes are overlapped, we give up on continuous collision.
                if (distanceOutput.distance <= 0.0f)
                {
                    // Failure!
                    output.state = TOIOutputState.Overlapped;
                    output.t = 0.0f;
                    break;
                }

                if (distanceOutput.distance < target + tolerance)
                {
                    // Victory!
                    output.state = TOIOutputState.Touching;
                    output.t = t1;
                    break;
                }

                // Initialize the separating axis.

                fcn.Initialize(cache, proxyA, sweepA, proxyB, sweepB, t1);

                // Compute the TOI on the separating axis. We do this by successively
                // resolving the deepest point. This loop is bounded by the number of vertices.
                var done = false;
                float t2 = tMax;
                var pushBackIter = 0;
                for (; ; )
                {
                    // Find the deepest point at t2. Store the witness point indices.
                    float s2 = fcn.FindMinSeparation(out int indexA, out int indexB, t2);

                    // Is the final configuration separated?
                    if (s2 > target + tolerance)
                    {
                        // Victory!
                        output.state = TOIOutputState.Separated;
                        output.t = tMax;
                        done = true;
                        break;
                    }

                    // Has the separation reached tolerance?
                    if (s2 > target - tolerance)
                    {
                        // Advance the sweeps
                        t1 = t2;
                        break;
                    }

                    // Compute the initial separation of the witness points.
                    float s1 = fcn.Evaluate(indexA, indexB, t1);

                    // Check for initial overlap. This might happen if the root finder
                    // runs out of iterations.
                    if (s1 < target - tolerance)
                    {
                        output.state = TOIOutputState.Failed;
                        output.t = t1;
                        done = true;
                        break;
                    }

                    // Check for touching
                    if (s1 <= target + tolerance)
                    {
                        // Victory! t1 should hold the TOI (could be 0.0).
                        output.state = TOIOutputState.Touching;
                        output.t = t1;
                        done = true;
                        break;
                    }

                    // Compute 1D root of: f(x) - target = 0
                    //int   rootIterCount = 0;
                    float a1 = t1, a2 = t2;
                    for (var rootIterCount = 0; rootIterCount < 50; ++rootIterCount)
                    {
                        // Use a mix of the secant rule and bisection.
                        float t;
                        if ((rootIterCount & 1) > 0)
                        // Secant rule to improve convergence.
                        {
                            t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
                        }
                        else
                        // Bisection to guarantee progress.
                        {
                            t = 0.5f * (a1 + a2);
                        }

                        //++rootIterCount;
                        //++b2_toiRootIters;

                        float s = fcn.Evaluate(indexA, indexB, t);

                        if (MathF.Abs(s - target) < tolerance)
                        {
                            // t2 holds a tentative value for t1
                            t2 = t;
                            break;
                        }

                        // Ensure we continue to bracket the root.
                        if (s > target)
                        {
                            a1 = t;
                            s1 = s;
                        }
                        else
                        {
                            a2 = t;
                            s2 = s;
                        }

                        // if (rootIterCount == 50) {
                        //   break;
                        // }
                    }

                    //b2_toiMaxRootIters = b2Max(b2_toiMaxRootIters, rootIterCount);

                    ++pushBackIter;

                    if (pushBackIter == Settings.MaxPolygonVertices)
                    {
                        break;
                    }
                }

                ++iter;
                //++b2_toiIters;

                if (done)
                {
                    break;
                }

                if (iter == Settings.MaxTOIIterations)
                {
                    // Root finder got stuck. Semi-victory.
                    output.state = TOIOutputState.Failed;
                    output.t = t1;
                    break;
                }
            }

            //_toiMaxIters = b2Max(b2_toiMaxIters, iter);

            // float time = timer.GetMilliseconds();
            // b2_toiMaxTime = b2Max(b2_toiMaxTime, time);
            // b2_toiTime += time;
        }
    }
}