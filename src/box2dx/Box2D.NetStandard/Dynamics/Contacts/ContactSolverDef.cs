namespace Box2DX.Dynamics {
  internal struct ContactSolverDef {
    internal TimeStep   step;
    internal Contact[]  contacts;
    internal int        count;
    internal Position[] positions;
    internal Velocity[] velocities;
  }
}