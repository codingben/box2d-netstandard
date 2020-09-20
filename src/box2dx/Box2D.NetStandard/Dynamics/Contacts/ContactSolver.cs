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

//#define B2_DEBUG_SOLVER

using System;
using System.Numerics;
using Box2D.NetStandard.Collision;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Fixtures;
using Box2D.NetStandard.Dynamics.World;
using Math = System.Math;

namespace Box2D.NetStandard.Dynamics.Contacts
{
	internal class ContactSolver
	{
		private readonly Contact[] _contacts;

		private readonly ContactPositionConstraint[] _positionConstraints;
		private readonly Position[] _positions;
		private readonly Velocity[] _velocities;
		public int _count;
		public TimeStep _step;

		internal ContactVelocityConstraint[] _velocityConstraints;

		public ContactSolver(ContactSolverDef def)
		{
			_step = def.step;
			_count = def.count;
			_positionConstraints = new ContactPositionConstraint[_count];
			_velocityConstraints = new ContactVelocityConstraint[_count];
			_positions = def.positions;
			_velocities = def.velocities;
			_contacts = def.contacts;

			for (var i = 0; i < _count; ++i)
			{
				Contact contact = _contacts[i];

				Fixture fixtureA = contact.m_fixtureA;
				Fixture fixtureB = contact.m_fixtureB;
				Shape shapeA = fixtureA.Shape;
				Shape shapeB = fixtureB.Shape;
				float radiusA = shapeA.m_radius;
				float radiusB = shapeB.m_radius;
				Body bodyA = fixtureA.Body;
				Body bodyB = fixtureB.Body;
				Manifold manifold = contact.Manifold;

				int pointCount = manifold.pointCount;
				//Debug.Assert(pointCount > 0);

				_velocityConstraints[i] = new ContactVelocityConstraint();
				ContactVelocityConstraint vc = _velocityConstraints[i];
				vc.friction = contact.m_friction;
				vc.restitution = contact.m_restitution;
				vc.tangentSpeed = contact.m_tangentSpeed;
				vc.indexA = bodyA.m_islandIndex;
				vc.indexB = bodyB.m_islandIndex;
				vc.invMassA = bodyA.m_invMass;
				vc.invMassB = bodyB.m_invMass;
				vc.invIA = bodyA.m_invI;
				vc.invIB = bodyB.m_invI;
				vc.contactIndex = i;
				vc.pointCount = pointCount;
				vc.K = new Matrix3x2();          // .SetZero();
				vc.normalMass = new Matrix3x2(); // .SetZero();

				_positionConstraints[i] = new ContactPositionConstraint();
				ContactPositionConstraint pc = _positionConstraints[i];
				pc.indexA = bodyA.m_islandIndex;
				pc.indexB = bodyB.m_islandIndex;
				pc.invMassA = bodyA.m_invMass;
				pc.invMassB = bodyB.m_invMass;
				pc.localCenterA = bodyA.m_sweep.localCenter;
				pc.localCenterB = bodyB.m_sweep.localCenter;
				pc.invIA = bodyA.m_invI;
				pc.invIB = bodyB.m_invI;
				pc.localNormal = manifold.localNormal;
				pc.localPoint = manifold.localPoint;
				pc.pointCount = pointCount;
				pc.radiusA = radiusA;
				pc.radiusB = radiusB;
				pc.type = manifold.type;

				for (var j = 0; j < pointCount; ++j)
				{
					ManifoldPoint cp = manifold.points[j];
					vc.points[j] = new VelocityConstraintPoint();
					VelocityConstraintPoint vcp = vc.points[j];

					if (_step.warmStarting)
					{
						vcp.normalImpulse = _step.dtRatio * cp.normalImpulse;
						vcp.tangentImpulse = _step.dtRatio * cp.tangentImpulse;
					}
					else
					{
						vcp.normalImpulse = 0f;
						vcp.tangentImpulse = 0f;
					}

					vcp.rA = Vector2.Zero;
					vcp.rB = Vector2.Zero;
					vcp.normalMass = 0f;
					vcp.tangentMass = 0f;
					vcp.velocityBias = 0f;

					pc.localPoints[j] = cp.localPoint;
				}
			}
		}

		public void InitializeVelocityConstraints()
		{
			for (var i = 0; i < _count; ++i)
			{
				ContactVelocityConstraint vc = _velocityConstraints[i];
				ContactPositionConstraint pc = _positionConstraints[i];

				float radiusA = pc.radiusA;
				float radiusB = pc.radiusB;
				Manifold manifold = _contacts[vc.contactIndex].Manifold;

				int indexA = vc.indexA;
				int indexB = vc.indexB;

				float mA = vc.invMassA;
				float mB = vc.invMassB;
				float iA = vc.invIA;
				float iB = vc.invIB;
				Vector2 localCenterA = pc.localCenterA;
				Vector2 localCenterB = pc.localCenterB;

				Vector2 cA = _positions[indexA].c;
				float aA = _positions[indexA].a;
				Vector2 vA = _velocities[indexA].v;
				float wA = _velocities[indexA].w;

				Vector2 cB = _positions[indexB].c;
				float aB = _positions[indexB].a;
				Vector2 vB = _velocities[indexB].v;
				float wB = _velocities[indexB].w;

				//Debug.Assert(manifold.pointCount > 0);

				var xfA = new Transform();
				var xfB = new Transform();

				xfA.q = Matrex.CreateRotation(aA);                   // Actually about twice as fast to use our own function
				xfB.q = Matrex.CreateRotation(aB);                   // Actually about twice as fast to use our own function
				xfA.p = cA - Vector2.Transform(localCenterA, xfA.q); // Common.Math.Mul(xfA.q, localCenterA);
				xfB.p = cB - Vector2.Transform(localCenterB, xfB.q); // Common.Math.Mul(xfB.q, localCenterB);

				var worldManifold = new WorldManifold();
				worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB);

				vc.normal = worldManifold.normal;

				int pointCount = vc.pointCount;
				for (var j = 0; j < pointCount; ++j)
				{
					VelocityConstraintPoint vcp = vc.points[j];

					vcp.rA = worldManifold.points[j] - cA;
					vcp.rB = worldManifold.points[j] - cB;

					float rnA = Vectex.Cross(vcp.rA, vc.normal);
					float rnB = Vectex.Cross(vcp.rB, vc.normal);

					float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

					vcp.normalMass = kNormal > 0f ? 1f / kNormal : 0f;

					Vector2 tangent = Vectex.Cross(vc.normal, 1f);

					float rtA = Vectex.Cross(vcp.rA, tangent);
					float rtB = Vectex.Cross(vcp.rB, tangent);

					float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

					vcp.tangentMass = kTangent > 0f ? 1f / kTangent : 0f;

					vcp.velocityBias = 0f;
					float vRel = Vector2.Dot(vc.normal, vB + Vectex.Cross(wB, vcp.rB) - vA - Vectex.Cross(wA, vcp.rA));
					if (vRel < -Settings.VelocityThreshold)
					{
						vcp.velocityBias = -vc.restitution * vRel;
					}
				}

				// If we have two points, then prepare the block solver.
				if (vc.pointCount == 2 && Settings.BlockSolve)
				{
					VelocityConstraintPoint vcp1 = vc.points[0];
					VelocityConstraintPoint vcp2 = vc.points[1];

					float rn1A = Vectex.Cross(vcp1.rA, vc.normal);
					float rn1B = Vectex.Cross(vcp1.rB, vc.normal);
					float rn2A = Vectex.Cross(vcp2.rA, vc.normal);
					float rn2B = Vectex.Cross(vcp2.rB, vc.normal);

					float k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
					float k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
					float k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

					// Ensure a reasonable condition number.
					const float k_maxConditionNumber = 1000.0f;
					if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
					{
						// K is safe to invert.
						vc.K = new Matrix3x2(k11, k12, k12, k22, 0, 0);

						// vc.K.ex       = new Vector2(k11, k12);
						// vc.K.ey       = new Vector2(k12, k22);
						/*Matrix3x2*/
						Matrex.Invert(vc.K, out Matrix3x2 KT);
						vc.normalMass = KT;
					}
					else
					{
						// The constraints are redundant, just use one.
						// TODO_ERIN use deepest?
						vc.pointCount = 1;
					}
				}
			}
		}

		public void WarmStart()
		{
			// Warm start.
			for (var i = 0; i < _count; ++i)
			{
				ContactVelocityConstraint vc = _velocityConstraints[i];

				int indexA = vc.indexA;
				int indexB = vc.indexB;
				float mA = vc.invMassA;
				float iA = vc.invIA;
				float mB = vc.invMassB;
				float iB = vc.invIB;
				int pointCount = vc.pointCount;

				Vector2 vA = _velocities[indexA].v;
				float wA = _velocities[indexA].w;
				Vector2 vB = _velocities[indexB].v;
				float wB = _velocities[indexB].w;

				Vector2 normal = vc.normal;
				Vector2 tangent = Vectex.Cross(normal, 1.0f);

				for (var j = 0; j < pointCount; ++j)
				{
					VelocityConstraintPoint vcp = vc.points[j];
					Vector2 P = vcp.normalImpulse * normal + vcp.tangentImpulse * tangent;
					wA -= iA * Vectex.Cross(vcp.rA, P);
					vA -= mA * P;
					wB += iB * Vectex.Cross(vcp.rB, P);
					vB += mB * P;
				}

				_velocities[indexA].v = vA;
				_velocities[indexA].w = wA;
				_velocities[indexB].v = vB;
				_velocities[indexB].w = wB;
			}
		}


		public void SolveVelocityConstraints()
		{
			for (var i = 0; i < _count; ++i)
			{
				ContactVelocityConstraint vc = _velocityConstraints[i];

				int indexA = vc.indexA;
				int indexB = vc.indexB;
				float mA = vc.invMassA;
				float iA = vc.invIA;
				float mB = vc.invMassB;
				float iB = vc.invIB;
				int pointCount = vc.pointCount;

				Vector2 vA = _velocities[indexA].v;
				float wA = _velocities[indexA].w;
				Vector2 vB = _velocities[indexB].v;
				float wB = _velocities[indexB].w;

				Vector2 normal = vc.normal;
				Vector2 tangent = Vectex.Cross(normal, 1.0f);
				float friction = vc.friction;

				//Debug.Assert(pointCount == 1 || pointCount == 2);

				// Solve tangent constraints first because non-penetration is more important
				// than friction.
				for (var j = 0; j < pointCount; ++j)
				{
					VelocityConstraintPoint vcp = vc.points[j];

					// Relative velocity at contact
					Vector2 dv = vB + Vectex.Cross(wB, vcp.rB) - vA - Vectex.Cross(wA, vcp.rA);

					// Compute tangent force
					float vt = Vector2.Dot(dv, tangent) - vc.tangentSpeed;
					float lambda = vcp.tangentMass * -vt;

					// b2Clamp the accumulated force
					float maxFriction = friction * vcp.normalImpulse;
					float newImpulse = Math.Clamp(vcp.tangentImpulse + lambda, -maxFriction, maxFriction);
					lambda = newImpulse - vcp.tangentImpulse;
					vcp.tangentImpulse = newImpulse;

					// Apply contact impulse
					Vector2 P = lambda * tangent;

					vA -= mA * P;
					wA -= iA * Vectex.Cross(vcp.rA, P);

					vB += mB * P;
					wB += iB * Vectex.Cross(vcp.rB, P);
				}

				// Solve normal constraints
				if (pointCount == 1 || Settings.BlockSolve == false)
				{
					for (var j = 0; j < pointCount; ++j)
					{
						VelocityConstraintPoint vcp = vc.points[j];

						// Relative velocity at contact
						Vector2 dv = vB + Vectex.Cross(wB, vcp.rB) - vA - Vectex.Cross(wA, vcp.rA);

						// Compute normal impulse
						float vn = Vector2.Dot(dv, normal);
						float lambda = -vcp.normalMass * (vn - vcp.velocityBias);

						// b2Clamp the accumulated impulse
						float newImpulse = MathF.Max(vcp.normalImpulse + lambda, 0.0f);
						lambda = newImpulse - vcp.normalImpulse;
						vcp.normalImpulse = newImpulse;

						// Apply contact impulse
						Vector2 P = lambda * normal;
						vA -= mA * P;
						wA -= iA * Vectex.Cross(vcp.rA, P);

						vB += mB * P;
						wB += iB * Vectex.Cross(vcp.rB, P);
					}
				}
				else
				{
					// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
					// Build the mini LCP for this contact patch
					//
					// vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
					//
					// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
					// b = vn0 - velocityBias
					//
					// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
					// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
					// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
					// solution that satisfies the problem is chosen.
					// 
					// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
					// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
					//
					// Substitute:
					// 
					// x = a + d
					// 
					// a := old total impulse
					// x := new total impulse
					// d := incremental impulse 
					//
					// For the current iteration we extend the formula for the incremental impulse
					// to compute the new total impulse:
					//
					// vn = A * d + b
					//    = A * (x - a) + b
					//    = A * x + b - A * a
					//    = A * x + b'
					// b' = b - A * a;

					VelocityConstraintPoint cp1 = vc.points[0];
					VelocityConstraintPoint cp2 = vc.points[1];

					var a = new Vector2(cp1.normalImpulse, cp2.normalImpulse);
					//Debug.Assert(a.X >= 0.0f && a.Y >= 0.0f);

					// Relative velocity at contact
					Vector2 dv1 = vB + Vectex.Cross(wB, cp1.rB) - vA - Vectex.Cross(wA, cp1.rA);
					Vector2 dv2 = vB + Vectex.Cross(wB, cp2.rB) - vA - Vectex.Cross(wA, cp2.rA);

					// Compute normal velocity
					float vn1 = Vector2.Dot(dv1, normal);
					float vn2 = Vector2.Dot(dv2, normal);

					var b = new Vector2(vn1 - cp1.velocityBias,
					                    vn2 - cp2.velocityBias);

					// Compute b'
					b -= Vector2.Transform(a, vc.K); // Common.Math.Mul(vc.K, a);

					//const float k_errorTol = 1e-3f;
					//B2_NOT_USED(k_errorTol);

					for (;;)
					{
						//
						// Case 1: vn = 0
						//
						// 0 = A * x + b'
						//
						// Solve for x:
						//
						// x = - inv(A) * b'
						//
						Vector2 x = -Vector2.Transform(b, vc.normalMass); //Common.Math.Mul(vc.normalMass, b);

						if (x.X >= 0.0f && x.Y >= 0.0f)
						{
							// Get the incremental impulse
							Vector2 d = x - a;

							// Apply incremental impulse
							Vector2 P1 = d.X * normal;
							Vector2 P2 = d.Y * normal;
							vA -= mA * (P1 + P2);
							wA -= iA * (Vectex.Cross(cp1.rA, P1) + Vectex.Cross(cp2.rA, P2));

							vB += mB * (P1 + P2);
							wB += iB * (Vectex.Cross(cp1.rB, P1) + Vectex.Cross(cp2.rB, P2));

							// Accumulate
							cp1.normalImpulse = x.X;
							cp2.normalImpulse = x.Y;

							break;
						}

						//
						// Case 2: vn1 = 0 and x2 = 0
						//
						//   0 = a11 * x1 + a12 * 0 + b1' 
						// vn2 = a21 * x1 + a22 * 0 + b2'
						//
						x.X = -cp1.normalMass * b.X;
						x.Y = 0.0f;
						vn1 = 0.0f;
						vn2 = vc.K.M22 * x.X + b.Y;
						if (x.X >= 0.0f && vn2 >= 0.0f)
						{
							// Get the incremental impulse
							Vector2 d = x - a;

							// Apply incremental impulse
							Vector2 P1 = d.X * normal;
							Vector2 P2 = d.Y * normal;
							vA -= mA * (P1 + P2);
							wA -= iA * (Vectex.Cross(cp1.rA, P1) + Vectex.Cross(cp2.rA, P2));

							vB += mB * (P1 + P2);
							wB += iB * (Vectex.Cross(cp1.rB, P1) + Vectex.Cross(cp2.rB, P2));

							// Accumulate
							cp1.normalImpulse = x.X;
							cp2.normalImpulse = x.Y;

							break;
						}


						//
						// Case 3: vn2 = 0 and x1 = 0
						//
						// vn1 = a11 * 0 + a12 * x2 + b1' 
						//   0 = a21 * 0 + a22 * x2 + b2'
						//
						x.X = 0.0f;
						x.Y = -cp2.normalMass * b.Y;
						vn1 = vc.K.M12 * x.Y + b.X;
						vn2 = 0.0f;

						if (x.Y >= 0.0f && vn1 >= 0.0f)
						{
							// Resubstitute for the incremental impulse
							Vector2 d = x - a;

							// Apply incremental impulse
							Vector2 P1 = d.X * normal;
							Vector2 P2 = d.Y * normal;
							vA -= mA * (P1 + P2);
							wA -= iA * (Vectex.Cross(cp1.rA, P1) + Vectex.Cross(cp2.rA, P2));

							vB += mB * (P1 + P2);
							wB += iB * (Vectex.Cross(cp1.rB, P1) + Vectex.Cross(cp2.rB, P2));

							// Accumulate
							cp1.normalImpulse = x.X;
							cp2.normalImpulse = x.Y;

							break;
						}

						//
						// Case 4: x1 = 0 and x2 = 0
						// 
						// vn1 = b1
						// vn2 = b2;
						x.X = 0.0f;
						x.Y = 0.0f;
						vn1 = b.X;
						vn2 = b.Y;

						if (vn1 >= 0.0f && vn2 >= 0.0f)
						{
							// Resubstitute for the incremental impulse
							Vector2 d = x - a;

							// Apply incremental impulse
							Vector2 P1 = d.X * normal;
							Vector2 P2 = d.Y * normal;
							vA -= mA * (P1 + P2);
							wA -= iA * (Vectex.Cross(cp1.rA, P1) + Vectex.Cross(cp2.rA, P2));

							vB += mB * (P1 + P2);
							wB += iB * (Vectex.Cross(cp1.rB, P1) + Vectex.Cross(cp2.rB, P2));

							// Accumulate
							cp1.normalImpulse = x.X;
							cp2.normalImpulse = x.Y;
						}

						// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
						break;
					}
				}

				_velocities[indexA].v = vA;
				_velocities[indexA].w = wA;
				_velocities[indexB].v = vB;
				_velocities[indexB].w = wB;
			}
		}

		public void StoreImpulses()
		{
			for (var i = 0; i < _count; ++i)
			{
				ContactVelocityConstraint vc = _velocityConstraints[i];
				Manifold manifold = _contacts[vc.contactIndex].Manifold;

				for (var j = 0; j < vc.pointCount; ++j)
				{
					manifold.points[j].normalImpulse = vc.points[j].normalImpulse;
					manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse;
				}
			}
		}


		public bool SolvePositionConstraints()
		{
			var minSeparation = 0.0f;

			for (var i = 0; i < _count; ++i)
			{
				ContactPositionConstraint pc = _positionConstraints[i];

				int indexA = pc.indexA;
				int indexB = pc.indexB;
				Vector2 localCenterA = pc.localCenterA;
				float mA = pc.invMassA;
				float iA = pc.invIA;
				Vector2 localCenterB = pc.localCenterB;
				float mB = pc.invMassB;
				float iB = pc.invIB;
				int pointCount = pc.pointCount;

				Vector2 cA = _positions[indexA].c;
				float aA = _positions[indexA].a;

				Vector2 cB = _positions[indexB].c;
				float aB = _positions[indexB].a;

				// Solve normal constraints
				for (var j = 0; j < pointCount; ++j)
				{
					var xfA = new Transform();
					var xfB = new Transform();
					xfA.q = Matrex.CreateRotation(aA);                   // Actually about twice as fast to use our own function
					xfB.q = Matrex.CreateRotation(aB);                   // Actually about twice as fast to use our own function
					xfA.p = cA - Vector2.Transform(localCenterA, xfA.q); // Common.Math.Mul(xfA.q, localCenterA);
					xfB.p = cB - Vector2.Transform(localCenterB, xfB.q); // Common.Math.Mul(xfB.q, localCenterB);

					var psm = new PositionSolverManifold();
					psm.Initialize(pc, xfA, xfB, j);
					Vector2 normal = psm.normal;

					Vector2 point = psm.point;
					float separation = psm.separation;

					Vector2 rA = point - cA;
					Vector2 rB = point - cB;

					// Track max constraint error.
					minSeparation = MathF.Min(minSeparation, separation);

					// Prevent large corrections and allow slop.
					float C = Math.Clamp(Settings.Baumgarte * (separation + Settings.LinearSlop), -Settings.MaxLinearCorrection,
					                     0.0f);

					// Compute the effective mass.
					float rnA = Vectex.Cross(rA, normal);
					float rnB = Vectex.Cross(rB, normal);
					float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

					// Compute normal impulse
					float impulse = K > 0.0f ? -C / K : 0.0f;

					Vector2 P = impulse * normal;

					cA -= mA * P;
					aA -= iA * Vectex.Cross(rA, P);

					cB += mB * P;
					aB += iB * Vectex.Cross(rB, P);
				}

				_positions[indexA].c = cA;
				_positions[indexA].a = aA;

				_positions[indexB].c = cB;
				_positions[indexB].a = aB;
			}

			// We can't expect minSpeparation >= -b2_linearSlop because we don't
			// push the separation above -b2_linearSlop.
			return minSeparation >= -3.0f * Settings.LinearSlop;
		}


		public bool SolveTOIPositionConstraints(int toiIndexA, int toiIndexB)
		{
			var minSeparation = 0.0f;

			for (var i = 0; i < _count; ++i)
			{
				ContactPositionConstraint pc = _positionConstraints[i];

				int indexA = pc.indexA;
				int indexB = pc.indexB;
				Vector2 localCenterA = pc.localCenterA;
				Vector2 localCenterB = pc.localCenterB;
				int pointCount = pc.pointCount;

				var mA = 0.0f;
				var iA = 0.0f;
				if (indexA == toiIndexA || indexA == toiIndexB)
				{
					mA = pc.invMassA;
					iA = pc.invIA;
				}

				var mB = 0.0f;
				var iB = 0.0f;
				if (indexB == toiIndexA || indexB == toiIndexB)
				{
					mB = pc.invMassB;
					iB = pc.invIB;
				}

				Vector2 cA = _positions[indexA].c;
				float aA = _positions[indexA].a;

				Vector2 cB = _positions[indexB].c;
				float aB = _positions[indexB].a;

				// Solve normal constraints
				for (var j = 0; j < pointCount; ++j)
				{
					var xfA = new Transform();
					var xfB = new Transform();
					xfA.q = Matrex.CreateRotation(aA);                   // Actually about twice as fast to use our own function
					xfB.q = Matrex.CreateRotation(aB);                   // Actually about twice as fast to use our own function
					xfA.p = cA - Vector2.Transform(localCenterA, xfA.q); // Common.Math.Mul(xfA.q, localCenterA);
					xfB.p = cB - Vector2.Transform(localCenterB, xfB.q); // Common.Math.Mul(xfB.q, localCenterB);

					var psm = new PositionSolverManifold();
					psm.Initialize(pc, xfA, xfB, j);
					Vector2 normal = psm.normal;

					Vector2 point = psm.point;
					float separation = psm.separation;

					Vector2 rA = point - cA;
					Vector2 rB = point - cB;

					// Track max constraint error.
					minSeparation = MathF.Min(minSeparation, separation);

					// Prevent large corrections and allow slop.
					float C = Math.Clamp(Settings.TOIBaumgarte * (separation + Settings.LinearSlop),
					                     -Settings.MaxLinearCorrection, 0.0f);

					// Compute the effective mass.
					float rnA = Vectex.Cross(rA, normal);
					float rnB = Vectex.Cross(rB, normal);
					float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

					// Compute normal impulse
					float impulse = K > 0.0f ? -C / K : 0.0f;

					Vector2 P = impulse * normal;

					cA -= mA * P;
					aA -= iA * Vectex.Cross(rA, P);

					cB += mB * P;
					aB += iB * Vectex.Cross(rB, P);
				}

				_positions[indexA].c = cA;
				_positions[indexA].a = aA;

				_positions[indexB].c = cB;
				_positions[indexB].a = aB;
			}

			// We can't expect minSpeparation >= -b2_linearSlop because we don't
			// push the separation above -b2_linearSlop.
			return minSeparation >= -1.5f * Settings.LinearSlop;
		}

		internal class PositionSolverManifold
		{
			internal Vector2 normal;
			internal Vector2 point;
			internal float separation;

			internal void Initialize(ContactPositionConstraint pc, Transform xfA, Transform xfB, int index)
			{
				//Debug.Assert(pc.pointCount > 0);

				switch (pc.type)
				{
					case ManifoldType.Circles: {
						Vector2 pointA = Common.Math.Mul(xfA, pc.localPoint);
						Vector2 pointB = Common.Math.Mul(xfB, pc.localPoints[0]);
						normal = Vector2.Normalize(pointB - pointA);
						point = 0.5f * (pointA + pointB);
						separation = Vector2.Dot(pointB - pointA, normal) - pc.radiusA - pc.radiusB;
						break;
					}
					case ManifoldType.FaceA: {
						normal = Vector2.Transform(pc.localNormal, xfA.q); // Common.Math.Mul(xfA.q, pc.localNormal);
						Vector2 planePoint = Common.Math.Mul(xfA, pc.localPoint);

						Vector2 clipPoint = Common.Math.Mul(xfB, pc.localPoints[index]);
						separation = Vector2.Dot(clipPoint - planePoint, normal) - pc.radiusA - pc.radiusB;
						point = clipPoint;

						break;
					}

					case ManifoldType.FaceB: {
						normal = Vector2.Transform(pc.localNormal, xfB.q); // Common.Math.Mul(xfB.q, pc.localNormal);
						Vector2 planePoint = Common.Math.Mul(xfB, pc.localPoint);

						Vector2 clipPoint = Common.Math.Mul(xfA, pc.localPoints[index]);
						separation = Vector2.Dot(clipPoint - planePoint, normal) - pc.radiusA - pc.radiusB;
						point = clipPoint;

						// Ensure normal points from A to B
						normal = -normal;
						break;
					}
				}
			}
		}
	}
}