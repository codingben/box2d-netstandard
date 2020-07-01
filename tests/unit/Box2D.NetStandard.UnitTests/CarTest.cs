using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Joints;
using Box2D.NetStandard.Dynamics.World;
using Box2D.WorldTests;
using Xunit;
using Xunit.Abstractions;

namespace Box2D.NetStandard.UnitTests
{
    public class CarTest
    {
        private ITestOutputHelper output;

        public CarTest(ITestOutputHelper output)
        {
            this.output = output;
        }

        [Fact]
        public void CarTest1()
        {
            World world = CarWorld.CreateWorld(out Body[] bodies, out Joint[] joints);

            for (int i = 0; i < 1400; i++)
            {
                world.Step(0.016f, 8, 3);
            }

            Assert.True(bodies[1].GetPosition().X > 275f);
            Assert.True(bodies[2].GetPosition().X > 225f);
            Assert.True(bodies[3].GetPosition().X > 225f);
            Assert.True(bodies[4].GetPosition().X > 225f);
            Assert.True(bodies[5].GetPosition().X > 225f);
            Assert.True(bodies[6].GetPosition().X > 225f);
            Assert.True(bodies[7].GetPosition().X > 275f);
            Assert.True(bodies[8].GetPosition().X > 275f);
        }
    }
}