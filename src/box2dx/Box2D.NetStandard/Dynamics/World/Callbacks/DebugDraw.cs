using System;
using Box2D.NetStandard.Common;

namespace Box2D.NetStandard.Dynamics.World.Callbacks {
  /// <summary>
  /// Implement and register this class with a b2World to provide debug drawing of physics
  /// entities in your game.
  /// </summary>
  public abstract class DebugDraw
  {
    [Flags]
    public enum DrawFlags
    {
      Shape        = 0x0001, // draw shapes
      Joint        = 0x0002, // draw joint connections
      CoreShape    = 0x0004, // draw core (TOI) shapes       // should be removed in this revision?
      Aabb         = 0x0008, // draw axis aligned bounding boxes
      Obb          = 0x0010, // draw oriented bounding boxes       // should be removed in this revision?
      Pair         = 0x0020, // draw broad-phase pairs
      CenterOfMass = 0x0040, // draw center of mass frame
      Controller   = 0x0080  // draw center of mass frame
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