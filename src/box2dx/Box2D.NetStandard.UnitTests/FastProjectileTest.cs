using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.World;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using Xunit;
using Xunit.Abstractions;

namespace Box2D.NetStandard.UnitTests
{
    public class FastProjectileTest
    {
        private ITestOutputHelper output;

        public FastProjectileTest(ITestOutputHelper output)
        {
            this.output = output;
        }

        private void DrawBody(Graphics graphics, Pen pen, Body body, float scale = 32.00f)
        {
            var fixture = body.GetFixtureList();

            while (fixture != null)
            {
                switch (fixture.Shape)
                {
                    case CircleShape circle:
                        graphics.DrawEllipse(pen, scale * (body.Position.X - circle.Radius), scale * (body.Position.Y - circle.Radius), scale * 2 * circle.Radius, scale * 2 * circle.Radius);
                        // this.lineBatch.DrawCircle(Vector2.Transform(circle.Center, transform), scale * circle.Radius, color: Color.FromRgba(0xFF, 0x00, 0x00, 0x80));
                        break;
                    case PolygonShape polygon:
                        var vertices = polygon.GetVertices();
                        if (vertices.Length > 1)
                        {
                            for (var i = 0; i < vertices.Length - 1; i++)
                            {
                                graphics.DrawLine(pen, scale * (body.Position.X + vertices[i].X), scale * (body.Position.Y + vertices[i].Y), scale * (body.Position.X + vertices[i + 1].X), scale * (body.Position.Y + vertices[i + 1].Y));
                            }
                            graphics.DrawLine(pen, scale * (body.Position.X + vertices[^1].X), scale * (body.Position.Y + vertices[^1].Y), scale * (body.Position.X + vertices[0].X), scale * (body.Position.Y + vertices[0].Y));
                        }
                        break;
                }
                fixture = fixture.Next;
            }
        }

        private void DrawScene(int frame, Body ship, IList<Body> asteroids, Body projectile)
        {
            Bitmap bitmap = new Bitmap(512, 512);

            using var graphics = Graphics.FromImage(bitmap);

            Pen pen = new Pen(System.Drawing.Color.Red);
            DrawBody(graphics, pen, ship);
            foreach (var asteroid in asteroids)
            {
                DrawBody(graphics, pen, asteroid);
            }
            DrawBody(graphics, pen, projectile);

            bitmap.Save($"frame_{frame}.png");
        }

        [Fact]
        public void FastProjectileTest1()
        {
            var world = new World(Vector2.Zero);

            var asteroids = new List<Body>();

            var initialAsteroidPosition = new Vector2(15.00f, 10.00f);
            var asteroidDef = new BodyDef
            {
                position = initialAsteroidPosition,
                type = BodyType.Dynamic,
                angularDamping = 1.00f,
                linearDamping = 0.05f,
            };
            var asteroid = world.CreateBody(asteroidDef);
            asteroid.CreateFixture(new CircleShape { Radius = 0.40f }, 10.00f);
            asteroids.Add(asteroid);
            var random = new Random(0);
            for (int i = 0; i < 512; i++)
            {
                var otherAsteroidDef = new BodyDef
                {
                    position = new Vector2(16 + random.Next(128) - 8, 16 + random.Next(128) - 8),
                    type = BodyType.Dynamic,
                    angularDamping = 1.00f,
                    linearDamping = 0.05f,
                    angle = (float)(random.NextDouble() * 2.00 * Math.PI),
                };
                var otherAsteroid = world.CreateBody(otherAsteroidDef);
                otherAsteroid.CreateFixture(new CircleShape { Radius = 0.40f }, 10.00f);
                asteroids.Add(otherAsteroid);
            }

            var initialShipPosition = new Vector2(10.00f, 10.00f);
            var shipDef = new BodyDef { position = initialShipPosition, type = BodyType.Dynamic, angularDamping = 10.00f, linearDamping = 0.50f };
            var ship = world.CreateBody(shipDef);

            var shipShape = new PolygonShape(new Vector2(-0.53125f, +0.53125f), new Vector2(+0.53125f, 0.00f), new Vector2(-0.53125f, -0.59375f));
            ship.CreateFixture(shipShape, 1.00f);

            var projectileVelocity = 30.00f * Vector2.UnitX + 0.01f * Vector2.UnitY;
            var projectileDef = new BodyDef
            {
                position = ship.Position,
                type = BodyType.Dynamic,
                linearVelocity = projectileVelocity / 10,
                bullet = true,
            };
            var projectile = world.CreateBody(projectileDef);
            projectile.CreateFixture(new CircleShape { Radius = 0.1f }, 1.00f);

            for (var i = 0; i < 200; i++)
            {
                world.Step(1.00f / 50.00f, 6, 2);
                if (i % 4 == 0)
                    DrawScene(i / 4, ship, asteroids, projectile);
            }

            // Assert.InRange(Vector2.Distance(initialShipPosition, ship.Position), -float.Epsilon, +float.Epsilon);
            Assert.NotInRange(Vector2.Distance(initialAsteroidPosition, asteroid.Position), -float.Epsilon, +float.Epsilon);
        }
    }
}
