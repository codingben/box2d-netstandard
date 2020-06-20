namespace Box2D.NetStandard.Collision {
  /// <summary>
  /// Contact ids to facilitate warm starting.
  /// </summary>
  [System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Explicit)]
  internal struct ContactID {
    [System.Runtime.InteropServices.FieldOffset(0)]
    internal ContactFeature cf;

    /// <summary>
    /// Used to quickly compare contact ids.
    /// </summary>
    [System.Runtime.InteropServices.FieldOffset(0)]
    internal uint key;
  }
}