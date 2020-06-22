using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Joints;
using Box2D.NetStandard.Dynamics.World;
using TestWorlds;
using Xunit;
using Xunit.Abstractions;

namespace Box2D.NetStandard.UnitTests {
  public class RubeGoldbergTest {
    private ITestOutputHelper output;


    public RubeGoldbergTest(ITestOutputHelper output) {
      this.output = output;
    }


    [Fact]
    public void Test1() {
      World world = RubeGoldberg.CreateWorld(out Body[] bodies, out Joint[] joints);
      for (int i = 0; i < 2100; i++)
        world.Step(0.016f, 8, 3);
    }
  }
}