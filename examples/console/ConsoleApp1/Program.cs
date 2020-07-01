using System;
using System.Threading;
using Box2DX.Collision;
using Box2DX.Common;
using Box2DX.Dynamics;

namespace ConsoleApp1
{
    public class Program
    {
        private static void Main()
        {
            var world = new World(
                new AABB()
                {
                    LowerBound = new Vec2(-100, -100),
                    UpperBound = new Vec2(100, 100)
                },
                new Vec2(0, -10),
                false);
            var groundDef = new BodyDef();
            groundDef.Position = new Vec2(0, -10);

            var ground = world.CreateBody(groundDef);
            ground.SetStatic();

            var groundFixtureDef = new EdgeDef();
            groundFixtureDef.Vertex1 = new Vec2(-100, 0);
            groundFixtureDef.Vertex2 = new Vec2(100, 0);
            ground.CreateFixture(groundFixtureDef);

            var ballDef = new BodyDef();
            ballDef.Position = new Vec2(0, 0);
            
            var ball = world.CreateBody(ballDef);

            var ballFixtureDef = new CircleDef();
            ballFixtureDef.LocalPosition = new Vec2(0, 0);
            ballFixtureDef.Radius = 0.25f;
            ballFixtureDef.Density = 1f;

            ball.CreateFixture(ballFixtureDef);
            ball.SetMassFromShapes();

            while (true)
            {
                while (ball.GetPosition().Y > -9.7)
                {
                    Console.WriteLine(ball.GetPosition().Y);

                    world.Step(0.02f, 8, 3);

                    Thread.Sleep(100);
                }

                ball.ApplyForce(new Vec2(0, 100), ball.GetPosition());
                
                world.Step(0.02f, 8, 3);
            }
        }
    }
}