using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Fixtures;
using Box2D.NetStandard.Dynamics.World;
using b2Vec2 = System.Numerics.Vector2;
using b2CircleShape = Box2D.NetStandard.Collision.Shapes.CircleShape;
using b2ChainShape = Box2D.NetStandard.Collision.Shapes.ChainShape;

namespace TestWorlds {
  public static class box2dBug604 {
    public static World CreateWorld() {
      b2Vec2 gravity = new b2Vec2(0.0f, -10.0f);
      World world = new World(gravity);
      b2ChainShape chainShape = new b2ChainShape();
      b2Vec2[] vertices =  {new b2Vec2(-5, 0), new b2Vec2(5, 0), new b2Vec2(5, 5), new b2Vec2(4, 1), new b2Vec2(-4, 1), new b2Vec2(-5, 5)};
      chainShape.CreateLoop(vertices);

      FixtureDef groundFixtureDef = new FixtureDef();
      groundFixtureDef.density = 0;
      groundFixtureDef.shape   = chainShape;

      BodyDef groundBodyDef = new BodyDef();
      groundBodyDef.type = BodyType.Static;

      Body     groundBody        = world.CreateBody(groundBodyDef);
      Fixture  groundBodyFixture = groundBody.CreateFixture(groundFixtureDef);

      b2CircleShape ballShape = new b2CircleShape();
      ballShape.Radius = 1;

      FixtureDef ballFixtureDef = new FixtureDef();
      ballFixtureDef.restitution = 0.75f;
      ballFixtureDef.density     = 1;
      ballFixtureDef.shape       = ballShape;

      BodyDef ballBodyDef = new BodyDef();
      ballBodyDef.type     = BodyType.Dynamic;
      ballBodyDef.position = new b2Vec2(0, 10);

      Body     ball        = world.CreateBody(ballBodyDef);
      Fixture  ballFixture = ball.CreateFixture(ballFixtureDef);
      ball.ApplyForceToCenter(new b2Vec2(-1000, -400), true);
      return world;
    }
  }
}