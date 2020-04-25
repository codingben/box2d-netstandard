using Box2DX.Dynamics;
using Box2DX.Common;
using Color = Box2DX.Dynamics.Color;

namespace HelloWorld
{
    public class DrawPhysics : DebugDraw
    {
        private readonly IPhysicsSimulationWindow physicsSimulationWindow;

        public DrawPhysics(IPhysicsSimulationWindow physicsSimulationWindow)
        {
            this.physicsSimulationWindow = physicsSimulationWindow;
        }

        public override void DrawPolygon(Vec2[] vertices, int vertexCount, Color color)
        {
            physicsSimulationWindow.DrawPolygon(vertices, vertexCount, color);
        }

        public override void DrawSolidPolygon(Vec2[] vertices, int vertexCount, Color color)
        {
            physicsSimulationWindow.DrawSolidPolygon(vertices, vertexCount, color);
        }

        public override void DrawCircle(Vec2 center, float radius, Color color)
        {
            physicsSimulationWindow.DrawCircle(center, radius, color);
        }

        public override void DrawSolidCircle(Vec2 center, float radius, Vec2 axis, Color color)
        {
            physicsSimulationWindow.DrawSolidCircle(center, radius, axis, color);
        }

        public override void DrawSegment(Vec2 p1, Vec2 p2, Color color)
        {
            physicsSimulationWindow.DrawSegment(p1, p2, color);
        }

        public override void DrawXForm(XForm xf)
        {
            physicsSimulationWindow.DrawXForm(xf);
        }
    }
}