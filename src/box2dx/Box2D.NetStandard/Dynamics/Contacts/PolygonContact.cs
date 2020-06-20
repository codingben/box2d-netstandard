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

using System.Diagnostics;
using Box2D.NetStandard.Collision;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;

namespace Box2D.NetStandard.Dynamics.Contacts
{
	public class PolygonContact : Contact
	{
		public PolygonContact(Fixture.Fixture fixtureA, Fixture.Fixture fixtureB)
			: base(fixtureA,0, fixtureB,0)
		{
			Debug.Assert(fixtureA.Type == ShapeType.Polygon);
			Debug.Assert(fixtureB.Type == ShapeType.Polygon);
		}

		
		public new static Contact Create(Fixture.Fixture fixtureA, Fixture.Fixture fixtureB)
		{
			return new PolygonContact(fixtureA, fixtureB);
		}

		public new static void Destroy(ref Contact contact)
		{
			contact = null;
		}

		internal override void Evaluate(out Manifold manifold, in Transform xfA, in Transform xfB) {
			Collision.Collision.CollidePolygons(out manifold, (PolygonShape)m_fixtureA.Shape, in xfA, (PolygonShape)m_fixtureB.Shape, in xfB);
		}
	}
}