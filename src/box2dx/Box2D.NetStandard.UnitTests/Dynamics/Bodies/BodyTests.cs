using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Fixtures;
using Box2D.NetStandard.Dynamics.World;
using Xunit;

namespace Box2D.NetStandard.UnitTests.Dynamics.Bodies
{
    public class BodyTests
    {
        private readonly Body body;

        public BodyTests()
        {
            World world = new World();
            body = world.CreateBody(new BodyDef());
        }

        [Fact]
        public void CanDestroyTopFixture()
        {
            FixtureDef fixtureDef = new FixtureDef() { shape = new PolygonShape(1, 1) };
            Fixture fixture1 = body.CreateFixture(fixtureDef);
            Fixture fixture2 = body.CreateFixture(fixtureDef);
            Fixture fixture3 = body.CreateFixture(fixtureDef);

            // Ensure that fixture3 is still the first in the linked list,
            // per current implementation
            Assert.Equal(fixture3, body.GetFixtureList());

            body.DestroyFixture(fixture3);

            Assert.Null(fixture3.Body);
            Assert.Null(fixture3.Next);
            Assert.Equal(fixture2, body.GetFixtureList());
            Assert.Equal(fixture1, fixture2.Next);
        }

        [Fact]
        public void CanDestroyMiddleFixture()
        {
            FixtureDef fixtureDef = new FixtureDef() { shape = new PolygonShape(1, 1) };
            Fixture fixture1 = body.CreateFixture(fixtureDef);
            Fixture fixture2 = body.CreateFixture(fixtureDef);
            Fixture fixture3 = body.CreateFixture(fixtureDef);

            // Ensure that fixture2 is the second in the linked list,
            // per current implementation
            Assert.Equal(fixture2, body.GetFixtureList().Next);

            body.DestroyFixture(fixture2);

            Assert.Null(fixture2.Body);
            Assert.Null(fixture2.Next);
            Assert.Equal(fixture3, body.GetFixtureList());
            Assert.Equal(fixture1, fixture3.Next);
        }

        [Fact]
        public void CanDestroyLastFixture()
        {
            FixtureDef fixtureDef = new FixtureDef() { shape = new PolygonShape(1, 1) };
            Fixture fixture1 = body.CreateFixture(fixtureDef);
            Fixture fixture2 = body.CreateFixture(fixtureDef);
            Fixture fixture3 = body.CreateFixture(fixtureDef);

            // Ensure that fixture1 is the third in the linked list,
            // per current implementation
            Assert.Equal(fixture1, body.GetFixtureList().Next.Next);

            body.DestroyFixture(fixture1);

            Assert.Null(fixture1.Body);
            Assert.Null(fixture1.Next);
            Assert.Equal(fixture3, body.GetFixtureList());
            Assert.Equal(fixture2, fixture3.Next);
        }
    }
}
