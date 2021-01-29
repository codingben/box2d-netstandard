using System;
using System.Collections.Generic;
using System.Linq;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Joints;
using Box2D.NetStandard.Dynamics.World;
using Box2D.WorldTests;
using Xunit;
using Xunit.Abstractions;

namespace Box2D.NetStandard.UnitTests
{
    public class RubeGoldbergTest
    {
        private ITestOutputHelper output;

        public RubeGoldbergTest(ITestOutputHelper output)
        {
            this.output = output;
        }

        [Fact]
        public void Test1()
        {
            World world = RubeGoldbergWorld.CreateWorld(out Body[] bodies, out Joint[] joints);

            for (int i = 0; i < 2100; i++)
            {
                world.Step(0.016f, 8, 3);
            }

            Assert.Equal(5.375222f, bodies[1].GetPosition().X);
            Assert.Equal(-6.7589993f, bodies[1].GetPosition().Y);
            Assert.Equal(6.088722f, bodies[2].GetPosition().X);
            Assert.Equal(-6.972047f, bodies[2].GetPosition().Y);
            Assert.Equal(5.316906f, bodies[3].GetPosition().X);
            Assert.Equal(-6.1546006f, bodies[3].GetPosition().Y);
            Assert.Equal(5.8592563f, bodies[4].GetPosition().X);
            Assert.Equal(-6.554182f, bodies[4].GetPosition().Y);
            Assert.Equal(5.780044f, bodies[5].GetPosition().X);
            Assert.Equal(-6.043272f, bodies[5].GetPosition().Y);
            Assert.Equal(5.4776254f, bodies[6].GetPosition().X);
            Assert.Equal(-5.3691053f, bodies[6].GetPosition().Y);
            Assert.Equal(5.813423f, bodies[7].GetPosition().X);
            Assert.Equal(-7.6760397f, bodies[7].GetPosition().Y);
            Assert.Equal(5.832724f, bodies[8].GetPosition().X);
            Assert.Equal(-6.2964587f, bodies[8].GetPosition().Y);
            Assert.Equal(6.1774282f, bodies[9].GetPosition().X);
            Assert.Equal(-5.8665905f, bodies[9].GetPosition().Y);
            Assert.Equal(5.6955895f, bodies[10].GetPosition().X);
            Assert.Equal(-5.7814617f, bodies[10].GetPosition().Y);
            Assert.Equal(5.5765705f, bodies[11].GetPosition().X);
            Assert.Equal(-5.543533f, bodies[11].GetPosition().Y);
            Assert.Equal(5.735431f, bodies[12].GetPosition().X);
            Assert.Equal(-6.9863505f, bodies[12].GetPosition().Y);
            Assert.Equal(6.1545367f, bodies[13].GetPosition().X);
            Assert.Equal(-7.6829896f, bodies[13].GetPosition().Y);
            Assert.Equal(6.354069f, bodies[14].GetPosition().X);
            Assert.Equal(-6.429799f, bodies[14].GetPosition().Y);
            Assert.Equal(-12.724164f, bodies[15].GetPosition().X);
            Assert.Equal(-6.725517f, bodies[15].GetPosition().Y);
            Assert.Equal(7.1534233f, bodies[16].GetPosition().X);
            Assert.Equal(1.6703408f, bodies[16].GetPosition().Y);
            Assert.Equal(21.839502f, bodies[17].GetPosition().X);
            Assert.Equal(3.6077049f, bodies[17].GetPosition().Y);
            Assert.Equal(-0.039129034f, bodies[19].GetPosition().X);
            Assert.Equal(-0.79102045f, bodies[19].GetPosition().Y);
            Assert.Equal(8.13909f, bodies[20].GetPosition().X);
            Assert.Equal(-1.2156444f, bodies[20].GetPosition().Y);
            Assert.Equal(16.557186f, bodies[21].GetPosition().X);
            Assert.Equal(0.6287131f, bodies[21].GetPosition().Y);
            Assert.Equal(9.629036f, bodies[22].GetPosition().X);
            Assert.Equal(6.00522f, bodies[22].GetPosition().Y);
            Assert.Equal(20.780727f, bodies[23].GetPosition().X);
            Assert.Equal(3.0136487f, bodies[23].GetPosition().Y);
            Assert.Equal(-0.2823656f, bodies[24].GetPosition().X);
            Assert.Equal(-1.3917149f, bodies[24].GetPosition().Y);
            Assert.Equal(1.6163651f, bodies[26].GetPosition().X);
            Assert.Equal(6.1943407f, bodies[26].GetPosition().Y);
            Assert.Equal(-11.750748f, bodies[27].GetPosition().X);
            Assert.Equal(-7.5754385f, bodies[27].GetPosition().Y);
            Assert.Equal(25.748251f, bodies[28].GetPosition().X);
            Assert.Equal(3.607715f, bodies[28].GetPosition().Y);
            Assert.Equal(26.783665f, bodies[29].GetPosition().X);
            Assert.Equal(3.607715f, bodies[29].GetPosition().Y);
            Assert.Equal(13.522972f, bodies[30].GetPosition().X);
            Assert.Equal(0.08209647f, bodies[30].GetPosition().Y);
            Assert.Equal(12.482032f, bodies[31].GetPosition().X);
            Assert.Equal(0.082096465f, bodies[31].GetPosition().Y);
            Assert.Equal(9.420522f, bodies[32].GetPosition().X);
            Assert.Equal(5.7002473f, bodies[32].GetPosition().Y);
            Assert.Equal(12.246641f, bodies[33].GetPosition().X);
            Assert.Equal(5.55919f, bodies[33].GetPosition().Y);
            Assert.Equal(15.596029f, bodies[34].GetPosition().X);
            Assert.Equal(0.08210217f, bodies[34].GetPosition().Y);
            Assert.Equal(8.146443f, bodies[35].GetPosition().X);
            Assert.Equal(8.501168f, bodies[35].GetPosition().Y);
            Assert.Equal(11.444387f, bodies[36].GetPosition().X);
            Assert.Equal(0.08209647f, bodies[36].GetPosition().Y);
            Assert.Equal(28.103752f, bodies[37].GetPosition().X);
            Assert.Equal(6.6315517f, bodies[37].GetPosition().Y);
            Assert.Equal(18.671846f, bodies[38].GetPosition().X);
            Assert.Equal(1.8235047f, bodies[38].GetPosition().Y);
            Assert.Equal(17.615961f, bodies[39].GetPosition().X);
            Assert.Equal(1.2294595f, bodies[39].GetPosition().Y);
            Assert.Equal(28.732674f, bodies[40].GetPosition().X);
            Assert.Equal(5.134416f, bodies[40].GetPosition().Y);
            Assert.Equal(-0.6563879f, bodies[41].GetPosition().X);
            Assert.Equal(-1.1420796f, bodies[41].GetPosition().Y);
            Assert.Equal(14.558385f, bodies[43].GetPosition().X);
            Assert.Equal(0.082096465f, bodies[43].GetPosition().Y);
            Assert.Equal(24.708204f, bodies[46].GetPosition().X);
            Assert.Equal(3.607715f, bodies[46].GetPosition().Y);
            Assert.Equal(22.756681f, bodies[47].GetPosition().X);
            Assert.Equal(3.5990844f, bodies[47].GetPosition().Y);
            Assert.Equal(19.727732f, bodies[49].GetPosition().X);
            Assert.Equal(2.419726f, bodies[49].GetPosition().Y);
        }
    }
}