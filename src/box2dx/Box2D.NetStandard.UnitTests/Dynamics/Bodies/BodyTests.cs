using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Fixtures;
using Box2D.NetStandard.Dynamics.World;
using Xunit;

namespace Box2D.NetStandard.UnitTests.Dynamics.Bodies
{
    public class BodyTests
    {
        private readonly Body sut;

        public BodyTests()
        {
            var world = new World();
            var bd = new BodyDef();

            sut = world.CreateBody(bd);
        }

        [Fact]
        public void CanDestroyTopFixture()
        {
            //arrange
            var fixtureDef = new FixtureDef() { shape = new PolygonShape(1, 1) };
            var fixture1 = sut.CreateFixture(fixtureDef);
            var fixture2 = sut.CreateFixture(fixtureDef);
            var fixture3 = sut.CreateFixture(fixtureDef);

            Assert.Equal(fixture3, sut.GetFixtureList()); //ensure that fixture3 is still the first in the linked list, per current implementation


            //act
            sut.DestroyFixture(fixture3);


            //assert
            Assert.Null(fixture3.Body);
            Assert.Null(fixture3.Next);
            Assert.Equal(fixture2, sut.GetFixtureList());
            Assert.Equal(fixture1, fixture2.Next);
        }

        [Fact]
        public void CanDestroyMiddleFixture()
        {
            //arrange
            var fixtureDef = new FixtureDef() { shape = new PolygonShape(1, 1) };
            var fixture1 = sut.CreateFixture(fixtureDef);
            var fixture2 = sut.CreateFixture(fixtureDef);
            var fixture3 = sut.CreateFixture(fixtureDef);

            Assert.Equal(fixture2, sut.GetFixtureList().Next); //ensure that fixture2 is the second in the linked list, per current implementation


            //act
            sut.DestroyFixture(fixture2);


            //assert
            Assert.Null(fixture2.Body);
            Assert.Null(fixture2.Next);
            Assert.Equal(fixture3, sut.GetFixtureList());
            Assert.Equal(fixture1, fixture3.Next);
        }

        [Fact]
        public void CanDestroyLastFixture()
        {
            //arrange
            var fixtureDef = new FixtureDef() { shape = new PolygonShape(1, 1) };
            var fixture1 = sut.CreateFixture(fixtureDef);
            var fixture2 = sut.CreateFixture(fixtureDef);
            var fixture3 = sut.CreateFixture(fixtureDef);

            Assert.Equal(fixture1, sut.GetFixtureList().Next.Next); //ensure that fixture1 is the third in the linked list, per current implementation


            //act
            sut.DestroyFixture(fixture1);


            //assert
            Assert.Null(fixture1.Body);
            Assert.Null(fixture1.Next);
            Assert.Equal(fixture3, sut.GetFixtureList());
            Assert.Equal(fixture2, fixture3.Next);
        }
    }
}
