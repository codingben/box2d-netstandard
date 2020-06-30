using System.Numerics;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.World;
using Microsoft.VisualStudio.TestPlatform.Utilities;
using Xunit;
using Xunit.Abstractions;

namespace Box2D.NetStandard.UnitTests
{
    public class SetLinearVelocityTest
    {
        private readonly ITestOutputHelper _output;

        public SetLinearVelocityTest(ITestOutputHelper output)
        {
            _output = output;
        }

        [Fact]
        public void TestOnlyOneBodyGetsVelocity()
        {
            World world = new World();

            Body ground = world.CreateBody(
              new BodyDef
              {
                  position = new Vector2(0, -1)
              });
            ground.CreateFixture(new EdgeShape(new Vector2(-100, 0), new Vector2(100, 0)));

            Body dynamic1 = world.CreateBody(
              new BodyDef
              {
                  position = new Vector2(-10, 3),
                  type = BodyType.Dynamic
              });
            dynamic1.CreateFixture(new PolygonShape(2, 2), 1f);

            Body dynamic2 = world.CreateBody(
              new BodyDef
              {
                  position = new Vector2(10, 3),
                  type = BodyType.Dynamic
              });
            dynamic2.CreateFixture(new PolygonShape(2, 2), 1f);

            // Let them fall to the floor
            world.Step(1000, 10, 10);

            Vector2 starting1 = dynamic1.GetPosition();
            Vector2 starting2 = dynamic2.GetPosition();

            // Apply force to one object so it should move away from the other
            dynamic1.SetLinearVelocity(new Vector2(-10, 0));

            world.Step(100, 10, 10);

            Vector2 ending1 = dynamic1.GetPosition();
            Vector2 ending2 = dynamic2.GetPosition();

            // It should not have moved vertically
            Assert.True((starting1.Y - ending1.Y) < 0.01f);
            // It should have moved left
            Assert.True(ending1.X < starting1.X);

            // Body 2 should not have moved
            Assert.Equal(starting2, ending2);
        }
    }
}