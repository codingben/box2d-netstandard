using Box2D.NetStandard.Dynamics.World;

namespace Box2D.NetStandard.Dynamics.Contacts {
  internal struct ContactSolverDef {
    internal TimeStep   step;
    internal Contact[]  contacts;
    internal int        count;
    internal Position[] positions;
    internal Velocity[] velocities;
  }
}