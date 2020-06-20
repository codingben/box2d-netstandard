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
using Box2D.NetStandard.Collision;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Contacts;
using Box2D.NetStandard.Dynamics.Fixtures;
using Box2D.NetStandard.Dynamics.Joints;

#pragma warning disable 618

namespace Box2D.NetStandard.Dynamics
{
	/// <summary>
	/// Joints and shapes are destroyed when their associated
	/// body is destroyed. Implement this listener so that you
	/// may nullify references to these joints and shapes.
	/// </summary>
	public abstract class DestructionListener
	{
		/// <summary>
		/// Called when any joint is about to be destroyed due
		/// to the destruction of one of its attached bodies.
		/// </summary>
		public abstract void SayGoodbye(Joint joint);

		/// <summary>
		/// Called when any shape is about to be destroyed due
		/// to the destruction of its parent body.
		/// </summary>
		public abstract void SayGoodbye(Fixture fixture);
	}

	/// <summary>
	/// Implement this class to provide collision filtering. In other words, you can implement
	/// this class if you want finer control over contact creation.
	/// </summary>
	public class ContactFilter
	{
		/// <summary>
		/// Return true if contact calculations should be performed between these two shapes.
		/// If you implement your own collision filter you may want to build from this implementation.
		/// @warning for performance reasons this is only called when the AABBs begin to overlap.
		/// </summary>
		public virtual bool ShouldCollide(Fixture fixtureA, Fixture fixtureB)
		{
			Filter filterA = fixtureA.m_filter;
			Filter filterB = fixtureB.m_filter;

			if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0)
				return filterA.groupIndex > 0;

			bool collide = (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0;
			return collide;
		}

		/// <summary>
		/// Return true if the given shape should be considered for ray intersection.
		/// </summary>
		public bool RayCollide(object userData, Fixture fixture)
		{
			//By default, cast userData as a shape, and then collide if the shapes would collide
			if (userData == null)
			{
				return true;
			}

			return ShouldCollide((Fixture)userData, fixture);
		}
	}

	/// Contact impulses for reporting. Impulses are used instead of forces because
	/// sub-step forces may approach infinity for rigid body collisions. These
	/// match up one-to-one with the contact points in b2Manifold.
	public class ContactImpulse
	{
		public float[] normalImpulses = new float[Settings.MaxManifoldPoints];
		public float[] tangentImpulses = new float[Settings.MaxManifoldPoints];
	}

	/// Implement this class to get contact information. You can use these results for
	/// things like sounds and game logic. You can also get contact results by
	/// traversing the contact lists after the time step. However, you might miss
	/// some contacts because continuous physics leads to sub-stepping.
	/// Additionally you may receive multiple callbacks for the same contact in a
	/// single time step.
	/// You should strive to make your callbacks efficient because there may be
	/// many callbacks per time step.
	/// @warning You cannot create/destroy Box2DX entities inside these callbacks.
	public interface ContactListener
	{
		/// Called when two fixtures begin to touch.
		void BeginContact(in Contact contact);

		/// Called when two fixtures cease to touch.
		void EndContact(in Contact contact);

		/// This is called after a contact is updated. This allows you to inspect a
		/// contact before it goes to the solver. If you are careful, you can modify the
		/// contact manifold (e.g. disable contact).
		/// A copy of the old manifold is provided so that you can detect changes.
		/// Note: this is called only for awake bodies.
		/// Note: this is called even when the number of contact points is zero.
		/// Note: this is not called for sensors.
		/// Note: if you set the number of contact points to zero, you will not
		/// get an EndContact callback. However, you may get a BeginContact callback
		/// the next step.
		void PreSolve(in Contact contact, in Manifold oldManifold);

		/// This lets you inspect a contact after the solver is finished. This is useful
		/// for inspecting impulses.
		/// Note: the contact manifold does not include time of impact impulses, which can be
		/// arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
		/// in a separate data structure.
		/// Note: this is only called for contacts that are touching, solid, and awake.
		void PostSolve(in Contact contact, in ContactImpulse impulse);
	}

	/// <summary>
	/// Color for debug drawing. Each value has the range [0,1].
	/// </summary>
	public struct Color
	{
		public float R, G, B;

		public Color(float r, float g, float b)
		{
			R = r; G = g; B = b;
		}
		public void Set(float r, float g, float b)
		{
			R = r; G = g; B = b;
		}
	}

	/// <summary>
	/// Implement and register this class with a b2World to provide debug drawing of physics
	/// entities in your game.
	/// </summary>
	public abstract class DebugDraw
	{
		[Flags]
		public enum DrawFlags
		{
			Shape = 0x0001, // draw shapes
			Joint = 0x0002, // draw joint connections
			CoreShape = 0x0004, // draw core (TOI) shapes       // should be removed in this revision?
			Aabb = 0x0008, // draw axis aligned bounding boxes
			Obb = 0x0010, // draw oriented bounding boxes       // should be removed in this revision?
			Pair = 0x0020, // draw broad-phase pairs
			CenterOfMass = 0x0040, // draw center of mass frame
			Controller = 0x0080 // draw center of mass frame
		};

		protected DrawFlags _drawFlags;

		public DebugDraw()
		{
			_drawFlags = 0;
		}

		public DrawFlags Flags { get { return _drawFlags; } set { _drawFlags = value; } }

		/// <summary>
		/// Append flags to the current flags.
		/// </summary>
		public void AppendFlags(DrawFlags flags)
		{
			_drawFlags |= flags;
		}

		/// <summary>
		/// Clear flags from the current flags.
		/// </summary>
		public void ClearFlags(DrawFlags flags)
		{
			_drawFlags &= ~flags;
		}

		/// <summary>
		/// Draw a closed polygon provided in CCW order.
		/// </summary>
		public abstract void DrawPolygon(Vec2[] vertices, int vertexCount, Color color);
		
		/// <summary>
		/// Draw a solid closed polygon provided in CCW order.
		/// </summary>
		public abstract void DrawSolidPolygon(Vec2[] vertices, int vertexCount, Color color);
		
		/// <summary>
		/// Draw a circle.
		/// </summary>
		public abstract void DrawCircle(Vec2 center, float radius, Color color);

		/// <summary>
		/// Draw a solid circle.
		/// </summary>
		public abstract void DrawSolidCircle(Vec2 center, float radius, Vec2 axis, Color color);

		/// <summary>
		/// Draw a line segment.
		/// </summary>
		public abstract void DrawSegment(Vec2 p1, Vec2 p2, Color color);

		/// <summary>
		/// Draw a transform. Choose your own length scale.
		/// </summary>
		/// <param name="xf">A transform.</param>
		public abstract void DrawXForm(Transform xf);
	}
}
