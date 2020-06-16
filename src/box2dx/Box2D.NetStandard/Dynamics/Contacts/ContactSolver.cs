/*
  Box2DX Copyright (c) 2009 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

//#define B2_DEBUG_SOLVER

using System;
using System.Diagnostics;
using System.Numerics;
using Box2DX.Collision;
using Box2DX.Common;

namespace Box2DX.Dynamics
{
	public struct ContactConstraintPoint
	{
		public Vector2 LocalPoint;
		public Vector2 RA;
		public Vector2 RB;
		public float NormalImpulse;
		public float TangentImpulse;
		public float NormalMass;
		public float TangentMass;
		public float EqualizedMass;
		public float VelocityBias;
	}

	public class ContactConstraint
	{
		public ContactConstraintPoint[] Points = new ContactConstraintPoint[Settings.MaxManifoldPoints];
		public Vector2 LocalPlaneNormal;
		public Vector2 LocalPoint;
		public Vector2 Normal;
		public Mat22 NormalMass;
		public Mat22 K;
		public Body BodyA;
		public Body BodyB;
		public ManifoldType Type;
		public float Radius;
		public float Friction;
		public float Restitution;
		public int PointCount;
		public Manifold Manifold;

		//public ContactConstraint()
		//{
		//	for (int i = 0; i < Settings.MaxManifoldPoints; i++)
		//		Points[i] = new ContactConstraintPoint();
		//}
	}

	public class ContactSolver : IDisposable
	{
		public TimeStep _step;
		public ContactConstraint[] _constraints;
		public int _constraintCount;

		public ContactSolver(TimeStep step, Contact[] contacts, int contactCount)
		{
			_step = step;
			_constraintCount = contactCount;

			_constraints = new ContactConstraint[_constraintCount];
			for (int i = 0; i < _constraintCount; i++)
				_constraints[i] = new ContactConstraint();

			for (int i = 0; i < _constraintCount; ++i)
			{
				Contact contact = contacts[i];

				Fixture fixtureA = contact._fixtureA;
				Fixture fixtureB = contact._fixtureB;
				Shape shapeA = fixtureA.Shape;
				Shape shapeB = fixtureB.Shape;
				float radiusA = shapeA._radius;
				float radiusB = shapeB._radius;
				Body bodyA = fixtureA.Body;
				Body bodyB = fixtureB.Body;
				Manifold manifold = contact.Manifold;

				float friction = Settings.MixFriction(fixtureA.Friction, fixtureB.Friction);
				float restitution = Settings.MixRestitution(fixtureA.Restitution, fixtureB.Restitution);

				Vector2 vA = bodyA._linearVelocity;
				Vector2 vB = bodyB._linearVelocity;
				float wA = bodyA._angularVelocity;
				float wB = bodyB._angularVelocity;

				Debug.Assert(manifold.PointCount > 0);

				WorldManifold worldManifold = new WorldManifold();
				worldManifold.Initialize(manifold, bodyA._xf, radiusA, bodyB._xf, radiusB);

				ContactConstraint cc = _constraints[i];
				cc.BodyA = bodyA;
				cc.BodyB = bodyB;
				cc.Manifold = manifold;
				cc.Normal = worldManifold.Normal;
				cc.PointCount = manifold.PointCount;
				cc.Friction = friction;
				cc.Restitution = restitution;

				cc.LocalPlaneNormal = manifold.LocalPlaneNormal;
				cc.LocalPoint = manifold.LocalPoint;
				cc.Radius = radiusA + radiusB;
				cc.Type = manifold.Type;

				unsafe
				{
					fixed (ContactConstraintPoint* ccPointsPtr = cc.Points)
					{
						for (int j = 0; j < cc.PointCount; ++j)
						{
							ManifoldPoint cp = manifold.Points[j];
							ContactConstraintPoint* ccp = &ccPointsPtr[j];

							ccp->NormalImpulse = cp.NormalImpulse;
							ccp->TangentImpulse = cp.TangentImpulse;

							ccp->LocalPoint = cp.LocalPoint;

							ccp->RA = worldManifold.Points[j] - bodyA._sweep.C;
							ccp->RB = worldManifold.Points[j] - bodyB._sweep.C;

							float rnA = Vectex.Cross(ccp->RA, cc.Normal);
							float rnB = Vectex.Cross(ccp->RB, cc.Normal);
							rnA *= rnA;
							rnB *= rnB;

							float kNormal = bodyA._invMass + bodyB._invMass + bodyA._invI * rnA + bodyB._invI * rnB;

							Debug.Assert(kNormal > Settings.FLT_EPSILON);
							ccp->NormalMass = 1.0f / kNormal;

							float kEqualized = bodyA._mass * bodyA._invMass + bodyB._mass * bodyB._invMass;
							kEqualized += bodyA._mass * bodyA._invI * rnA + bodyB._mass * bodyB._invI * rnB;

							Debug.Assert(kEqualized > Settings.FLT_EPSILON);
							ccp->EqualizedMass = 1.0f / kEqualized;

							Vector2 tangent = Vectex.Cross(cc.Normal, 1.0f);

							float rtA = Vectex.Cross(ccp->RA, tangent);
							float rtB = Vectex.Cross(ccp->RB, tangent);
							rtA *= rtA;
							rtB *= rtB;

							float kTangent = bodyA._invMass + bodyB._invMass + bodyA._invI * rtA + bodyB._invI * rtB;

							Debug.Assert(kTangent > Settings.FLT_EPSILON);
							ccp->TangentMass = 1.0f / kTangent;

							// Setup a velocity bias for restitution.
							ccp->VelocityBias = 0.0f;
							float vRel = Vector2.Dot(cc.Normal, vB + Vectex.Cross(wB, ccp->RB) - vA - Vectex.Cross(wA, ccp->RA));
							if (vRel < -Settings.VelocityThreshold)
							{
								ccp->VelocityBias = -cc.Restitution * vRel;
							}
						}

						// If we have two points, then prepare the block solver.
						if (cc.PointCount == 2)
						{
							ContactConstraintPoint* ccp1 = &ccPointsPtr[0];
							ContactConstraintPoint* ccp2 = &ccPointsPtr[1];

							float invMassA = bodyA._invMass;
							float invIA = bodyA._invI;
							float invMassB = bodyB._invMass;
							float invIB = bodyB._invI;

							float rn1A = Vectex.Cross(ccp1->RA, cc.Normal);
							float rn1B = Vectex.Cross(ccp1->RB, cc.Normal);
							float rn2A = Vectex.Cross(ccp2->RA, cc.Normal);
							float rn2B = Vectex.Cross(ccp2->RB, cc.Normal);

							float k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
							float k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
							float k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;

							// Ensure a reasonable condition number.
							const float k_maxConditionNumber = 100.0f;
							if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
							{
								// K is safe to invert.
								cc.K.Col1=new Vector2(k11, k12);
								cc.K.Col2=new Vector2(k12, k22);
								cc.NormalMass = cc.K.GetInverse();
							}
							else
							{
								// The constraints are redundant, just use one.
								// TODO_ERIN use deepest?
								cc.PointCount = 1;
							}
						}
					}
				}
			}
		}

		public void Dispose()
		{
			_constraints = null;
		}

		public void InitVelocityConstraints(TimeStep step)
		{
			unsafe
			{
				// Warm start.
				for (int i = 0; i < _constraintCount; ++i)
				{
					ContactConstraint c = _constraints[i];

					Body bodyA = c.BodyA;
					Body bodyB = c.BodyB;
					float invMassA = bodyA._invMass;
					float invIA = bodyA._invI;
					float invMassB = bodyB._invMass;
					float invIB = bodyB._invI;
					Vector2 normal = c.Normal;
					Vector2 tangent = Vectex.Cross(normal, 1.0f);

					fixed (ContactConstraintPoint* pointsPtr = c.Points)
					{
						if (step.WarmStarting)
						{
							for (int j = 0; j < c.PointCount; ++j)
							{
								ContactConstraintPoint* ccp = &pointsPtr[j];
								ccp->NormalImpulse *= step.DtRatio;
								ccp->TangentImpulse *= step.DtRatio;
								Vector2 P = ccp->NormalImpulse * normal + ccp->TangentImpulse * tangent;
								bodyA._angularVelocity -= invIA * Vectex.Cross(ccp->RA, P);
								bodyA._linearVelocity -= invMassA * P;
								bodyB._angularVelocity += invIB * Vectex.Cross(ccp->RB, P);
								bodyB._linearVelocity += invMassB * P;
							}
						}
						else
						{
							for (int j = 0; j < c.PointCount; ++j)
							{
								ContactConstraintPoint* ccp = &pointsPtr[j];
								ccp->NormalImpulse = 0.0f;
								ccp->TangentImpulse = 0.0f;
							}
						}
					}
				}
			}
		}

		public void SolveVelocityConstraints()
		{
			for (int i = 0; i < _constraintCount; ++i)
			{
				ContactConstraint c = _constraints[i];
				Body bodyA = c.BodyA;
				Body bodyB = c.BodyB;
				float wA = bodyA._angularVelocity;
				float wB = bodyB._angularVelocity;
				Vector2 vA = bodyA._linearVelocity;
				Vector2 vB = bodyB._linearVelocity;
				float invMassA = bodyA._invMass;
				float invIA = bodyA._invI;
				float invMassB = bodyB._invMass;
				float invIB = bodyB._invI;
				Vector2 normal = c.Normal;
				Vector2 tangent = Vectex.Cross(normal, 1.0f);
				float friction = c.Friction;

				Debug.Assert(c.PointCount == 1 || c.PointCount == 2);

				unsafe
				{
					fixed (ContactConstraintPoint* pointsPtr = c.Points)
					{
						// Solve tangent constraints
						for (int j = 0; j < c.PointCount; ++j)
						{
							ContactConstraintPoint* ccp = &pointsPtr[j];

							// Relative velocity at contact
							Vector2 dv = vB + Vectex.Cross(wB, ccp->RB) - vA - Vectex.Cross(wA, ccp->RA);

							// Compute tangent force
							float vt = Vector2.Dot(dv, tangent);
							float lambda = ccp->TangentMass * (-vt);

							// b2Clamp the accumulated force
							float maxFriction = friction * ccp->NormalImpulse;
							float newImpulse = System.Math.Clamp(ccp->TangentImpulse + lambda, -maxFriction, maxFriction);
							lambda = newImpulse - ccp->TangentImpulse;

							// Apply contact impulse
							Vector2 P = lambda * tangent;

							vA -= invMassA * P;
							wA -= invIA * Vectex.Cross(ccp->RA, P);

							vB += invMassB * P;
							wB += invIB * Vectex.Cross(ccp->RB, P);

							ccp->TangentImpulse = newImpulse;
						}

						// Solve normal constraints
						if (c.PointCount == 1)
						{
							ContactConstraintPoint ccp = c.Points[0];

							// Relative velocity at contact
							Vector2 dv = vB + Vectex.Cross(wB, ccp.RB) - vA - Vectex.Cross(wA, ccp.RA);

							// Compute normal impulse
							float vn = Vector2.Dot(dv, normal);
							float lambda = -ccp.NormalMass * (vn - ccp.VelocityBias);

							// Clamp the accumulated impulse
							float newImpulse = MathF.Max(ccp.NormalImpulse + lambda, 0.0f);
							lambda = newImpulse - ccp.NormalImpulse;

							// Apply contact impulse
							Vector2 P = lambda * normal;
							vA -= invMassA * P;
							wA -= invIA * Vectex.Cross(ccp.RA, P);

							vB += invMassB * P;
							wB += invIB * Vectex.Cross(ccp.RB, P);
							ccp.NormalImpulse = newImpulse;
						}
						else
						{
							// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
							// Build the mini LCP for this contact patch
							//
							// vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
							//
							// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
							// b = vn_0 - velocityBias
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
							// x = x' - a
							// 
							// Plug into above equation:
							//
							// vn = A * x + b
							//    = A * (x' - a) + b
							//    = A * x' + b - A * a
							//    = A * x' + b'
							// b' = b - A * a;

							ContactConstraintPoint* cp1 = &pointsPtr[0];
							ContactConstraintPoint* cp2 = &pointsPtr[1];

							Vector2 a = new Vector2(cp1->NormalImpulse, cp2->NormalImpulse);
							Debug.Assert(a.X >= 0.0f && a.Y >= 0.0f);

							// Relative velocity at contact
							Vector2 dv1 = vB + Vectex.Cross(wB, cp1->RB) - vA - Vectex.Cross(wA, cp1->RA);
							Vector2 dv2 = vB + Vectex.Cross(wB, cp2->RB) - vA - Vectex.Cross(wA, cp2->RA);

							// Compute normal velocity
							float vn1 = Vector2.Dot(dv1, normal);
							float vn2 = Vector2.Dot(dv2, normal);

							Vector2 b;
							b.X = vn1 - cp1->VelocityBias;
							b.Y = vn2 - cp2->VelocityBias;
							b -= Common.Math.Mul(c.K, a);

							const float k_errorTol = 1e-3f;
							//B2_NOT_USED(k_errorTol);

							for (; ; )
							{
								//
								// Case 1: vn = 0
								//
								// 0 = A * x' + b'
								//
								// Solve for x':
								//
								// x' = - inv(A) * b'
								//
								Vector2 x = -Common.Math.Mul(c.NormalMass, b);

								if (x.X >= 0.0f && x.Y >= 0.0f)
								{
									// Resubstitute for the incremental impulse
									Vector2 d = x - a;

									// Apply incremental impulse
									Vector2 P1 = d.X * normal;
									Vector2 P2 = d.Y * normal;
									vA -= invMassA * (P1 + P2);
									wA -= invIA * (Vectex.Cross(cp1->RA, P1) + Vectex.Cross(cp2->RA, P2));

									vB += invMassB * (P1 + P2);
									wB += invIB * (Vectex.Cross(cp1->RB, P1) + Vectex.Cross(cp2->RB, P2));

									// Accumulate
									cp1->NormalImpulse = x.X;
									cp2->NormalImpulse = x.Y;

#if DEBUG_SOLVER
									// Postconditions
									dv1 = vB + Vectex.Cross(wB, cp1->RB) - vA - Vectex.Cross(wA, cp1->RA);
									dv2 = vB + Vectex.Cross(wB, cp2->RB) - vA - Vectex.Cross(wA, cp2->RA);

									// Compute normal velocity
									vn1 = Vector2.Dot(dv1, normal);
									vn2 = Vector2.Dot(dv2, normal);

									Box2DXDebug.Assert(Common.Math.Abs(vn1 - cp1.VelocityBias) < k_errorTol);
									Box2DXDebug.Assert(Common.Math.Abs(vn2 - cp2.VelocityBias) < k_errorTol);
#endif
									break;
								}

								//
								// Case 2: vn1 = 0 and x2 = 0
								//
								//   0 = a11 * x1' + a12 * 0 + b1' 
								// vn2 = a21 * x1' + a22 * 0 + b2'
								//
								x.X = -cp1->NormalMass * b.X;
								x.Y = 0.0f;
								vn1 = 0.0f;
								vn2 = c.K.Col1.Y * x.X + b.Y;

								if (x.X >= 0.0f && vn2 >= 0.0f)
								{
									// Resubstitute for the incremental impulse
									Vector2 d = x - a;

									// Apply incremental impulse
									Vector2 P1 = d.X * normal;
									Vector2 P2 = d.Y * normal;
									vA -= invMassA * (P1 + P2);
									wA -= invIA * (Vectex.Cross(cp1->RA, P1) + Vectex.Cross(cp2->RA, P2));

									vB += invMassB * (P1 + P2);
									wB += invIB * (Vectex.Cross(cp1->RB, P1) + Vectex.Cross(cp2->RB, P2));

									// Accumulate
									cp1->NormalImpulse = x.X;
									cp2->NormalImpulse = x.Y;

#if DEBUG_SOLVER
									// Postconditions
									dv1 = vB + Vectex.Cross(wB, cp1->RB) - vA - Vectex.Cross(wA, cp1->RA);

									// Compute normal velocity
									vn1 = Vector2.Dot(dv1, normal);

									Box2DXDebug.Assert(Common.Math.Abs(vn1 - cp1.VelocityBias) < k_errorTol);
#endif
									break;
								}


								//
								// Case 3: w2 = 0 and x1 = 0
								//
								// vn1 = a11 * 0 + a12 * x2' + b1' 
								//   0 = a21 * 0 + a22 * x2' + b2'
								//
								x.X = 0.0f;
								x.Y = -cp2->NormalMass * b.Y;
								vn1 = c.K.Col2.X * x.Y + b.X;
								vn2 = 0.0f;

								if (x.Y >= 0.0f && vn1 >= 0.0f)
								{
									// Resubstitute for the incremental impulse
									Vector2 d = x - a;

									// Apply incremental impulse
									Vector2 P1 = d.X * normal;
									Vector2 P2 = d.Y * normal;
									vA -= invMassA * (P1 + P2);
									wA -= invIA * (Vectex.Cross(cp1->RA, P1) + Vectex.Cross(cp2->RA, P2));

									vB += invMassB * (P1 + P2);
									wB += invIB * (Vectex.Cross(cp1->RB, P1) + Vectex.Cross(cp2->RB, P2));

									// Accumulate
									cp1->NormalImpulse = x.X;
									cp2->NormalImpulse = x.Y;

#if DEBUG_SOLVER
									// Postconditions
									dv2 = vB + Vectex.Cross(wB, cp2->RB) - vA - Vectex.Cross(wA, cp2->RA);

									// Compute normal velocity
									vn2 = Vector2.Dot(dv2, normal);

									Box2DXDebug.Assert(Common.Math.Abs(vn2 - cp2.VelocityBias) < k_errorTol);
#endif
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
									vA -= invMassA * (P1 + P2);
									wA -= invIA * (Vectex.Cross(cp1->RA, P1) + Vectex.Cross(cp2->RA, P2));

									vB += invMassB * (P1 + P2);
									wB += invIB * (Vectex.Cross(cp1->RB, P1) + Vectex.Cross(cp2->RB, P2));

									// Accumulate
									cp1->NormalImpulse = x.X;
									cp2->NormalImpulse = x.Y;

									break;
								}

								// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
								break;
							}
						}

						bodyA._linearVelocity = vA;
						bodyA._angularVelocity = wA;
						bodyB._linearVelocity = vB;
						bodyB._angularVelocity = wB;
					}
				}
			}
		}

		public void FinalizeVelocityConstraints()
		{
			for (int i = 0; i < _constraintCount; ++i)
			{
				ContactConstraint c = _constraints[i];
				Manifold m = c.Manifold;

				for (int j = 0; j < c.PointCount; ++j)
				{
					m.Points[j].NormalImpulse = c.Points[j].NormalImpulse;
					m.Points[j].TangentImpulse = c.Points[j].TangentImpulse;
				}
			}
		}

		internal class PositionSolverManifold
		{
			internal Vector2 Normal;
			internal Vector2[] Points = new Vector2[Settings.MaxManifoldPoints];
			internal float[] Separations = new float[Settings.MaxManifoldPoints];

			internal void Initialize(ContactConstraint cc)
			{
				Debug.Assert(cc.PointCount > 0);

				switch (cc.Type)
				{
					case ManifoldType.Circles:
						{
							Vector2 pointA = cc.BodyA.GetWorldPoint(cc.LocalPoint);
							Vector2 pointB = cc.BodyB.GetWorldPoint(cc.Points[0].LocalPoint);
							if (Vector2.DistanceSquared(pointA, pointB) > Settings.FLT_EPSILON_SQUARED)
							{
								Normal = Vector2.Normalize(pointB - pointA);
							}
							else
							{
								Normal = new Vector2(1.0f, 0.0f);
							}

							Points[0] = 0.5f * (pointA + pointB);
							Separations[0] = Vector2.Dot(pointB - pointA, Normal) - cc.Radius;
						}
						break;

					case ManifoldType.FaceA:
						{
							Normal = cc.BodyA.GetWorldVector(cc.LocalPlaneNormal);
							Vector2 planePoint = cc.BodyA.GetWorldPoint(cc.LocalPoint);

							for (int i = 0; i < cc.PointCount; ++i)
							{
								Vector2 clipPoint = cc.BodyB.GetWorldPoint(cc.Points[i].LocalPoint);
								Separations[i] = Vector2.Dot(clipPoint - planePoint, Normal) - cc.Radius;
								Points[i] = clipPoint;
							}
						}
						break;

					case ManifoldType.FaceB:
						{
							Normal = cc.BodyB.GetWorldVector(cc.LocalPlaneNormal);
							Vector2 planePoint = cc.BodyB.GetWorldPoint(cc.LocalPoint);

							for (int i = 0; i < cc.PointCount; ++i)
							{
								Vector2 clipPoint = cc.BodyA.GetWorldPoint(cc.Points[i].LocalPoint);
								Separations[i] = Vector2.Dot(clipPoint - planePoint, Normal) - cc.Radius;
								Points[i] = clipPoint;
							}

							// Ensure normal points from A to B
							Normal = -Normal;
						}
						break;
				}
			}
		}

		private static PositionSolverManifold s_PositionSolverManifold = new PositionSolverManifold();

		public bool SolvePositionConstraints(float baumgarte)
		{
			float minSeparation = 0.0f;

			for (int i = 0; i < _constraintCount; ++i)
			{
				ContactConstraint c = _constraints[i];
				Body bodyA = c.BodyA;
				Body bodyB = c.BodyB;

				float invMassA = bodyA._mass * bodyA._invMass;
				float invIA = bodyA._mass * bodyA._invI;
				float invMassB = bodyB._mass * bodyB._invMass;
				float invIB = bodyB._mass * bodyB._invI;

				s_PositionSolverManifold.Initialize(c);
				Vector2 normal = s_PositionSolverManifold.Normal;

				// Solver normal constraints
				for (int j = 0; j < c.PointCount; ++j)
				{
					Vector2 point = s_PositionSolverManifold.Points[j];
					float separation = s_PositionSolverManifold.Separations[j];

					Vector2 rA = point - bodyA._sweep.C;
					Vector2 rB = point - bodyB._sweep.C;

					// Track max constraint error.
					minSeparation = MathF.Min(minSeparation, separation);

					// Prevent large corrections and allow slop.
					float C = baumgarte * System.Math.Clamp(separation + Settings.LinearSlop, -Settings.MaxLinearCorrection, 0.0f);

					// Compute normal impulse
					float impulse = -c.Points[j].EqualizedMass * C;

					Vector2 P = impulse * normal;

					bodyA._sweep.C -= invMassA * P;
					bodyA._sweep.A -= invIA * Vectex.Cross(rA, P);
					bodyA.SynchronizeTransform();

					bodyB._sweep.C += invMassB * P;
					bodyB._sweep.A += invIB * Vectex.Cross(rB, P);
					bodyB.SynchronizeTransform();
				}
			}

			// We can't expect minSpeparation >= -Settings.LinearSlop because we don't
			// push the separation above -Settings.LinearSlop.
			return minSeparation >= -1.5f * Settings.LinearSlop;
		}
	}
}
