using System;
using System.Collections.Generic;
using System.Linq;
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
      
      
      Assert.Equal(18.522697f,   bodies[1].GetPosition().X);
      Assert.Equal(-7.1123757f,  bodies[1].GetPosition().Y);
      Assert.Equal(18.135706f,   bodies[2].GetPosition().X);
      Assert.Equal(-7.775176f,   bodies[2].GetPosition().Y);
      Assert.Equal(19.123291f,   bodies[3].GetPosition().X);
      Assert.Equal(-7.1213202f,  bodies[3].GetPosition().Y);
      Assert.Equal(18.541561f,   bodies[4].GetPosition().X);
      Assert.Equal(-7.550102f,   bodies[4].GetPosition().Y);
      Assert.Equal(19.058691f,   bodies[5].GetPosition().X);
      Assert.Equal(-7.5408025f,  bodies[5].GetPosition().Y);
      Assert.Equal(19.789547f,   bodies[6].GetPosition().X);
      Assert.Equal(-7.623065f,   bodies[6].GetPosition().Y);
      Assert.Equal(17.432444f,   bodies[7].GetPosition().X);
      Assert.Equal(-7.645199f,   bodies[7].GetPosition().Y);
      Assert.Equal(18.80003f,    bodies[8].GetPosition().X);
      Assert.Equal(-7.532749f,   bodies[8].GetPosition().Y);
      Assert.Equal(19.066462f,   bodies[9].GetPosition().X);
      Assert.Equal(-7.9689965f,  bodies[9].GetPosition().Y);
      Assert.Equal(19.333649f,   bodies[10].GetPosition().X);
      Assert.Equal(-7.556802f,   bodies[10].GetPosition().Y);
      Assert.Equal(19.597675f,   bodies[11].GetPosition().X);
      Assert.Equal(-7.56287f,    bodies[11].GetPosition().Y);
      Assert.Equal(18.102705f,   bodies[12].GetPosition().X);
      Assert.Equal(-7.461681f,   bodies[12].GetPosition().Y);
      Assert.Equal(17.431849f,   bodies[13].GetPosition().X);
      Assert.Equal(-7.8914f,     bodies[13].GetPosition().Y);
      Assert.Equal(18.4715f,     bodies[14].GetPosition().X);
      Assert.Equal(-7.980177f,   bodies[14].GetPosition().Y);
      Assert.Equal(-9.962016f,   bodies[15].GetPosition().X);
      Assert.Equal(-6.728336f,   bodies[15].GetPosition().Y);
      Assert.Equal(7.1534233f,   bodies[16].GetPosition().X);
      Assert.Equal(1.6703408f,   bodies[16].GetPosition().Y);
      Assert.Equal(22.840824f,   bodies[17].GetPosition().X);
      Assert.Equal(0.062232375f, bodies[17].GetPosition().Y);
      Assert.Equal(5.9275694f,   bodies[19].GetPosition().X);
      Assert.Equal(-5.8913975f,  bodies[19].GetPosition().Y);
      Assert.Equal(7.8510795f,   bodies[20].GetPosition().X);
      Assert.Equal(-1.2156444f,  bodies[20].GetPosition().Y);
      Assert.Equal(17.176409f,   bodies[21].GetPosition().X);
      Assert.Equal(0.43643707f,  bodies[21].GetPosition().Y);
      Assert.Equal(11.124716f,   bodies[22].GetPosition().X);
      Assert.Equal(5.832632f,    bodies[22].GetPosition().Y);
      Assert.Equal(21.401531f,   bodies[23].GetPosition().X);
      Assert.Equal(2.8061125f,   bodies[23].GetPosition().Y);
      Assert.Equal(-0.4141534f,  bodies[24].GetPosition().X);
      Assert.Equal(-1.718581f,   bodies[24].GetPosition().Y);
      Assert.Equal(0.8919028f,   bodies[26].GetPosition().X);
      Assert.Equal(5.3602657f,   bodies[26].GetPosition().Y);
      Assert.Equal(-0.20063685f, bodies[27].GetPosition().X);
      Assert.Equal(5.469171f,    bodies[27].GetPosition().Y);
      Assert.Equal(26.679678f,   bodies[28].GetPosition().X);
      Assert.Equal(2.8592424f,   bodies[28].GetPosition().Y);
      Assert.Equal(20.70694f,    bodies[29].GetPosition().X);
      Assert.Equal(-7.9673986f,  bodies[29].GetPosition().Y);
      Assert.Equal(14.428536f,   bodies[30].GetPosition().X);
      Assert.Equal(-0.40969118f, bodies[30].GetPosition().Y);
      Assert.Equal(13.4044695f,  bodies[31].GetPosition().X);
      Assert.Equal(-0.46029496f, bodies[31].GetPosition().Y);
      Assert.Equal(9.744946f,    bodies[32].GetPosition().X);
      Assert.Equal(5.702799f,    bodies[32].GetPosition().Y);
      Assert.Equal(12.246655f,   bodies[33].GetPosition().X);
      Assert.Equal(5.559356f,    bodies[33].GetPosition().Y);
      Assert.Equal(16.27186f,    bodies[34].GetPosition().X);
      Assert.Equal(-0.11289443f, bodies[34].GetPosition().Y);
      Assert.Equal(10.716021f,   bodies[35].GetPosition().X);
      Assert.Equal(-1.805789f,   bodies[35].GetPosition().Y);
      Assert.Equal(12.368353f,   bodies[36].GetPosition().X);
      Assert.Equal(-0.4810508f,  bodies[36].GetPosition().Y);
      Assert.Equal(9.17481f,     bodies[37].GetPosition().X);
      Assert.Equal(-2.0716884f,  bodies[37].GetPosition().Y);
      Assert.Equal(19.36593f,    bodies[38].GetPosition().X);
      Assert.Equal(1.6167523f,   bodies[38].GetPosition().Y);
      Assert.Equal(18.226791f,   bodies[39].GetPosition().X);
      Assert.Equal(0.9769168f,   bodies[39].GetPosition().Y);
      Assert.Equal(28.732674f,   bodies[40].GetPosition().X);
      Assert.Equal(5.134416f,    bodies[40].GetPosition().Y);
      Assert.Equal(-0.6240424f,  bodies[41].GetPosition().X);
      Assert.Equal(-1.1397804f,  bodies[41].GetPosition().Y);
      Assert.Equal(15.402854f,   bodies[43].GetPosition().X);
      Assert.Equal(-0.3022539f,  bodies[43].GetPosition().Y);
      Assert.Equal(25.655275f,   bodies[46].GetPosition().X);
      Assert.Equal(3.03654f,     bodies[46].GetPosition().Y);
      Assert.Equal(24.378824f,   bodies[47].GetPosition().X);
      Assert.Equal(3.33594f,     bodies[47].GetPosition().Y);
      Assert.Equal(20.37384f,    bodies[49].GetPosition().X);
      Assert.Equal(2.2022865f,   bodies[49].GetPosition().Y);
    }
  }
}