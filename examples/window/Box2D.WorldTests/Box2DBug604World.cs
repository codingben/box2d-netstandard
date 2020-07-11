using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Fixtures;
using Box2D.NetStandard.Dynamics.World;
using b2Vec2 = System.Numerics.Vector2;
using b2CircleShape = Box2D.NetStandard.Collision.Shapes.CircleShape;
using b2ChainShape = Box2D.NetStandard.Collision.Shapes.ChainShape;

namespace Box2D.WorldTests
{
    public static class Box2DBug604World
    {
        public static World CreateWorld()
        {
            var gravity = new b2Vec2(0.0f, -10.0f);
            var world = new World(gravity);
            var chainShape = new b2ChainShape();

            b2Vec2[] vertices =
            {
                new b2Vec2(-5, 0),
                new b2Vec2(5, 0),
                new b2Vec2(5, 5),
                new b2Vec2(4, 1),
                new b2Vec2(-4, 1),
                new b2Vec2(-5, 5)
            };

            chainShape.CreateLoop(vertices);

            var groundFixtureDef = new FixtureDef();
            groundFixtureDef.density = 0;
            groundFixtureDef.shape = chainShape;

            var groundBodyDef = new BodyDef();
            groundBodyDef.type = BodyType.Static;

            var groundBody = world.CreateBody(groundBodyDef);
            var groundBodyFixture = groundBody.CreateFixture(groundFixtureDef);

            var ballShape = new b2CircleShape();
            ballShape.Radius = 1;

            var ballFixtureDef = new FixtureDef();
            ballFixtureDef.restitution = 0.75f;
            ballFixtureDef.density = 1;
            ballFixtureDef.shape = ballShape;

            var ballBodyDef = new BodyDef();
            ballBodyDef.type = BodyType.Dynamic;
            ballBodyDef.position = new b2Vec2(0, 10);

            var ball = world.CreateBody(ballBodyDef);
            var ballFixture = ball.CreateFixture(ballFixtureDef);

            ball.ApplyForceToCenter(new b2Vec2(-1000, -400), true);

            return world;
        }
    }
}
