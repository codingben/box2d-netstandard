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

        public override void DrawPolygon(in Vec2[] vertices, int vertexCount, in Color color)
        {
            window.DrawPolygon(vertices, vertexCount, color);
        }

        public override void DrawSolidPolygon(in Vec2[] vertices, int vertexCount, in Color color)
        {
            window.DrawSolidPolygon(vertices, vertexCount, color);
        }

        public override void DrawCircle(in Vec2 center, float radius, in Color color)
        {
            window.DrawCircle(center, radius, color);
        }

        public override void DrawSolidCircle(in Vec2 center, float radius, in Vec2 axis, in Color color)
        {
            window.DrawSolidCircle(center, radius, axis, color);
        }

        public override void DrawSegment(in Vec2 p1, in Vec2 p2, in Color color)
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