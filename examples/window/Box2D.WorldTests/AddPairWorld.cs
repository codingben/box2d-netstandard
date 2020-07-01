using System;
using System.Numerics;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.World;

namespace Box2D.WorldTests
{
    // AddPair stress test as per Box2D Testbed
    public class AddPairWorld
    {
        private static Random random = new Random();

        public static World CreateWorld()
        {
            var world = new World(Vector2.Zero);
            var circle = new CircleShape();
            circle.Radius = 0.1f;

            var minX = -6f;
            var maxX = 0f;
            var minY = 4f;
            var maxY = 6f;
            var xRange = maxX - minX;
            var yRange = maxY - minY;

            for (int i = 0; i < 400; ++i)
            {
                var bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(
                    (float)(random.NextDouble() * xRange + minX),
                    (float)(random.NextDouble() * yRange + minY));

                var body = world.CreateBody(bd);
                body.CreateFixture(circle, 0.01f);
            }

            var polygon = new PolygonShape(1.5f, 1.5f);
            var bd2 = new BodyDef();
            bd2.type = BodyType.Dynamic;
            bd2.position = new Vector2(-40f, 5f);
            bd2.bullet = true;

            var body2 = world.CreateBody(bd2);
            body2.CreateFixture(polygon, 1f);
            body2.SetLinearVelocity(new Vector2(150, 0));

            return world;
        }
    }
}