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

#define DEBUG

namespace Box2D.NetStandard.Collision {
  /// <summary>
  /// Used to warm start Distance.
  /// Set count to zero on first call.
  /// </summary>
  public class SimplexCache {
    internal float metric;		///< length or area
    internal ushort count;

    internal  byte[] indexA = new byte[3];	///< vertices on shape A
    internal  byte[] indexB = new byte[3]; ///< vertices on shape B
  }

  // GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
}