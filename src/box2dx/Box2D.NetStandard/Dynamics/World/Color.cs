namespace Box2D.NetStandard.Dynamics.World {
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
}