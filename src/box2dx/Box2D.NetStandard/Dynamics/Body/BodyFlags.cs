using System;

namespace Box2DX.Dynamics {
  [Flags]
  public enum BodyFlags
  {
    
    Island        = 0x01,
    Awake         = 0x02,
    AutoSleep     = 0x04,
    Bullet        = 0x08,
    FixedRotation = 0x10,
    Enabled       = 0x20,
    TOI           = 0x40
  }
}