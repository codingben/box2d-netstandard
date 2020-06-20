namespace Box2D.NetStandard.Dynamics.World {
  internal struct TimeStep {
    internal float dt;      // time step
    internal float inv_dt;  // inverse time step (0 if dt == 0).
    internal float dtRatio; // dt * inv_dt0
    internal int   velocityIterations;
    internal int   positionIterations;
    internal bool  warmStarting;
  }
}