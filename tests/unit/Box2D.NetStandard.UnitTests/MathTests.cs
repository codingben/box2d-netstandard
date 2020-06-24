using System.Diagnostics;
using System.Numerics;
using System.Timers;
using Box2D.NetStandard.Common;
using Xunit;
using Xunit.Abstractions;

namespace Box2D.NetStandard.UnitTests {
  public class MathTests {
    private ITestOutputHelper output;


    public MathTests(ITestOutputHelper output) {
      this.output = output;
    }

    [Fact]
    public void Test1() {
      for (int i = 0; i < 100; i++) {
        Matrix3x2 ex1 = Matrix3x2.CreateRotation(i / Settings.Pi);
        Matrix3x2 ex2 = Matrex.CreateRotation(i    / Settings.Pi);
      }

      int iter = 1000000;

      for (int j = 0; j < 10; j++) {

        Stopwatch w = Stopwatch.StartNew();
        for (int i = 0; i < iter; i++) {
          Matrix3x2 ex1 = Matrix3x2.CreateRotation(i % 100 / Settings.Pi);

        }

        w.Stop();
        output.WriteLine("builtin " + w.ElapsedMilliseconds.ToString());

        w.Restart();
        for (int i = 0; i < iter; i++) {
          Matrix3x2 ex1 = Matrex.CreateRotation(i % 100 / Settings.Pi);

        }

        w.Stop();
        output.WriteLine("custom " + w.ElapsedMilliseconds.ToString());
      }
    }
  }
}