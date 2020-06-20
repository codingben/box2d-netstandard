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

using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;

namespace Box2D.NetStandard.Collision.Shapes {
  /// <summary>
  /// A shape is used for collision detection. You can create a shape however you like.
  /// Shapes used for simulation in World are created automatically when a Fixture is created.
  /// </summary>
  public abstract class Shape {
    public abstract Shape Clone();
    public abstract int   GetChildCount();
    public abstract bool  TestPoint(in Transform xf, in Vector2 p);

    public abstract bool RayCast(out RayCastOutput output, in RayCastInput input, in Transform transform,
                                    int childIndex);

    public abstract void ComputeAABB(out AABB aabb, in Transform xf, int childIndex);
    
    public abstract void ComputeMass(out MassData massData, float density);

    protected internal ShapeType m_type;

    public ShapeType Type {
      [MethodImpl(MethodImplOptions.AggressiveInlining)]
      get => m_type;
    }

    protected internal float m_radius;
    
  }
}