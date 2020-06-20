using System;

namespace Box2D.NetStandard.Dynamics.Contacts {
  [Flags]
  public enum CollisionFlags {
    Island    = 0x01,
    Touching  = 0x02,
    Enabled   = 0x04,
    Filter    = 0x08,
    BulletHit = 0x10,
    Toi       = 0x20
  }
}