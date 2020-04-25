using Box2DX.Common;
using Box2DX.Dynamics;

namespace HelloWorld
{
    public interface IPhysicsSimulationWindow
    {
        void DrawPolygon(Vec2[] vertices, int vertexCount, Color color);

        void DrawSolidPolygon(Vec2[] vertices, int vertexCount, Color color);

        void DrawCircle(Vec2 center, float radius, Color color);

        void DrawSolidCircle(Vec2 center, float radius, Vec2 axis, Color color);

        void DrawSegment(Vec2 p1, Vec2 p2, Color color);

        void DrawXForm(XForm xf);
    }
}