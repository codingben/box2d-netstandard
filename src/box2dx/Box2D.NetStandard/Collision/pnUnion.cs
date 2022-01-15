using System.Runtime.InteropServices;

namespace Box2D.NetStandard.Collision
{
    [StructLayout(LayoutKind.Explicit)]
    internal struct pnUnion
    {
        [FieldOffset(0)]
        internal int parent;

        [FieldOffset(0)]
        internal int next;
    }
}