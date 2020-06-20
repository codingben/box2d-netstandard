namespace Box2DX.Dynamics {
  public struct ContactRegister {
    public ContactCreateFcn  CreateFcn;
    public ContactDestroyFcn DestroyFcn;
    public bool              Primary;
  }
}