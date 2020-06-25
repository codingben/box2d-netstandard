using System;
using System.Numerics;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.World;

namespace TestWorlds {
  // AddPair stress test as per Box2D Testbed
  public class AddPair {
    private static Random r = new Random();
    
    public static World CreateWorld() {
      World world = new World(Vector2.Zero);
      
      CircleShape circle = new CircleShape();
      circle.m_radius = 0.1f;

      float minX = -6f;
      float maxX = 0f;
      float minY = 4f;
      float maxY = 6f;
      float xRange = maxX - minX;
      float yRange = maxY - minY;

      for (int i = 0; i < 400; ++i) {
        BodyDef bd = new BodyDef();
        bd.type = BodyType.Dynamic;
        bd.position = new Vector2((float) (r.NextDouble() * xRange + minX), (float) (r.NextDouble() * yRange + minY));
        Body body = world.CreateBody(bd);
        body.CreateFixture(circle, 0.01f);
      }
      
      PolygonShape polygon = new PolygonShape(1.5f,1.5f);
      BodyDef bd2 = new BodyDef();
      bd2.type = BodyType.Dynamic;
      bd2.position = new Vector2(-40f, 5f);
      bd2.bullet = true;
      Body body2 = world.CreateBody(bd2);
      body2.CreateFixture(polygon, 1f);
      body2.SetLinearVelocity(new Vector2(150,0));

      return world;
    }
    
  }
}