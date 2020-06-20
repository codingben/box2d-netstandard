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

namespace Box2D.NetStandard.Collision {
  // Structures and functions used for computing contact points, distance
  // queries, and TOI queries.

  public partial class Collision {
    public static bool TestOverlap(in AABB a, in AABB b) {
      Vector2 d1, d2;
      d1 = b.lowerBound - a.upperBound;
      d2 = a.lowerBound - b.upperBound;

      if (d1.X > 0.0f || d1.Y > 0.0f)
        return false;

      if (d2.X > 0.0f || d2.Y > 0.0f)
        return false;

      return true;
    }

    // Sutherland-Hodgman clipping.
    internal static int ClipSegmentToLine(out ClipVertex[ /*2*/] vOut, in ClipVertex[ /*2*/] vIn, in Vector2 normal,
       float                                                    offset, int vertexIndexA) {
      vOut = new ClipVertex[2];

      // Start with no output points
      int numOut = 0;

      // Calculate the distance of end points to the line
      float distance0 = Vector2.Dot(normal, vIn[0].v) - offset;
      float distance1 = Vector2.Dot(normal, vIn[1].v) - offset;

      // If the points are behind the plane
      if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
      if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

      // If the points are on different sides of the plane
      if (distance0 * distance1 < 0.0f)
      {
        // Find intersection point of edge and plane
        float interp = distance0 / (distance0 - distance1);
        vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);

        // VertexA is hitting edgeB.
        vOut[numOut].id.cf.indexA = (byte)(vertexIndexA);
        vOut[numOut].id.cf.indexB = vIn[0].id.cf.indexB;
        vOut[numOut].id.cf.typeA  = (byte)ContactFeatureType.Vertex;
        vOut[numOut].id.cf.typeB  = (byte)ContactFeatureType.Face;
        ++numOut;
      }

      return numOut;
    }
  }
}