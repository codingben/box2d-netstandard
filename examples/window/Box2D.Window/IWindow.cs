/*
    Window Simulation Copyright © Ben Ukhanov 2020
*/

using Box2DX.Common;
using Color = Box2DX.Dynamics.Color;

namespace Box2D.Window
{
    public interface IWindow
    {
        void DrawPolygon(Vec2[] vertices, int vertexCount, Color color);

        void DrawSolidPolygon(Vec2[] vertices, int vertexCount, Color color);

        void DrawCircle(Vec2 center, float radius, Color color);

        void DrawSolidCircle(Vec2 center, float radius, Vec2 axis, Color color);

        void DrawSegment(Vec2 p1, Vec2 p2, Color color);

        void DrawXForm(XForm xf);
    }
}