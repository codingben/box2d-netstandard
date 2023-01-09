/*
 * Window Simulation Copyright © Ben Ukhanov 2021
 */

using System.Numerics;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.World.Callbacks;
using Color = Box2D.NetStandard.Dynamics.World.Color;

namespace Box2D.Window
{
    public class DrawPhysics : DebugDraw
    {
        private readonly IWindow window;

        public DrawPhysics(IWindow window)
        {
            this.window = window;
        }

        public override void DrawPolygon(in Vector2[] vertices, int vertexCount, in Color color)
        {
            window.DrawPolygon(vertices, vertexCount, color);
        }

        public override void DrawSolidPolygon(in Vector2[] vertices, int vertexCount, in Color color)
        {
            window.DrawSolidPolygon(vertices, vertexCount, color);
        }

        public override void DrawCircle(in Vector2 center, float radius, in Color color)
        {
            window.DrawCircle(center, radius, color);
        }

        public override void DrawSolidCircle(in Vector2 center, float radius, in Vector2 axis, in Color color)
        {
            window.DrawSolidCircle(center, radius, axis, color);
        }

        public override void DrawSegment(in Vector2 p1, in Vector2 p2, in Color color)
        {
            window.DrawSegment(p1, p2, color);
        }

        public override void DrawTransform(in Transform xf)
        {
            window.DrawXForm(xf);
        }

        public override void DrawPoint(in Vector2 position, float size, in Color color)
        {
            // TODO: Draw points
        }
    }
}