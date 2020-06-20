namespace Box2D.NetStandard.Collision {
  /// <summary>
  /// Output parameters for b2TimeOfImpact.
  /// </summary>
  struct TOIOutput {
    internal TOIOutputState state;
    internal float          t;
  }
}