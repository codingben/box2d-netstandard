/*
    Window Simulation Copyright © Ben Ukhanov 2020
*/

using System.Numerics;
using Box2D.NetStandard.Common;
using Color = Box2D.NetStandard.Dynamics.World.Color;

namespace Box2D.Window
{
    public interface IWindow
    {
        void DrawPolygon(Vector2[] vertices, int vertexCount, Color color);

        void DrawSolidPolygon(Vector2[] vertices, int vertexCount, Color color);

        void DrawCircle(Vector2 center, float radius, Color color);

        void DrawSolidCircle(Vector2 center, float radius, Vector2 axis, Color color);

        void DrawSegment(Vector2 p1, Vector2 p2, Color color);

        void DrawXForm(Transform xf);
    }
}