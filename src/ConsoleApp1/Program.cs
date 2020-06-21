using System;
using System.Numerics;
using System.Threading;
using Box2DX.Collision;
using Box2DX.Dynamics;

namespace ConsoleApp1 {
  class Program {
    static void Main(string[] args) {
      World w = new World(new AABB(){LowerBound  = new  Vector2(-100,-100), UpperBound = new Vector2(100,100)},new Vector2(0,-10),false);
      var groundDef = new BodyDef();
      groundDef.Position=new Vector2(0,-10);
      Body ground = w.CreateBody(groundDef);
      ground.SetStatic();
      EdgeDef groundFixtureDef = new EdgeDef();
      groundFixtureDef.Vertex1=new Vector2(-100,0);
      groundFixtureDef.Vertex2=new Vector2(100,0);
      ground.CreateFixture(groundFixtureDef);

      BodyDef ballDef = new BodyDef();
      ballDef.Position=new Vector2(0,0); Body ball = w.CreateBody(ballDef);
      CircleDef ballFixtureDef = new CircleDef();
      ballFixtureDef.LocalPosition=new Vector2(0,0);
      ballFixtureDef.Radius = 0.25f;
      ballFixtureDef.Density = 1f;
      ball.CreateFixture(ballFixtureDef);
      ball.SetMassFromShapes();

      while (true) {

        while (ball.GetPosition().Y > -9.7) {

          Console.WriteLine(ball.GetPosition().Y);
          w.Step(0.02f, 8, 3);
          Thread.Sleep(100);
        }

        ball.ApplyForce(new Vector2(0,100),ball.GetPosition());
        w.Step(0.02f, 8, 3);

      }
    }
  }
}