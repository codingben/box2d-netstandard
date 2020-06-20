using System;
using System.Runtime.InteropServices;

namespace Box2DX.Collision {

  [StructLayout(LayoutKind.Explicit)]
  internal struct pnUnion {
    [FieldOffset(0)]
    internal Int32 parent;
    [FieldOffset(0)]
    internal Int32 next;
  }
  
  class TreeNode
  {
    internal bool IsLeaf() 
    {
      return child1 == -1;
    }

    /// Enlarged AABB
    
    
    internal pnUnion pn;

    internal AABB aabb;

    internal Int32 child1;
    internal Int32 child2;

    // leaf = 0, free node = -1
    internal Int32 height;

    internal bool moved;
    
    internal object userData;

  };
}