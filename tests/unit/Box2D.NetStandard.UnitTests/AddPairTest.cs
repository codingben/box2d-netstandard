using Box2D.NetStandard.Dynamics.World;
using Box2D.WorldTests;
using Xunit;
using MathF = System.Math;
using Float = System.Double;

namespace Box2D.NetStandard.UnitTests
{
    // For profiling
    public class AddPairTest
    {
        [Fact]
        public void AddPairs()
        {
            for (int j = 0; j < 1; j++)
            {
                World world = AddPairWorld.CreateWorld();

                for (int i = 0; i < 40; i++)
                {
                    world.Step(0.016f, 8, 3);
                }
            }
        }
    }
}