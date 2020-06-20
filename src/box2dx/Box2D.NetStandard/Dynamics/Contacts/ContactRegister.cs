namespace Box2D.NetStandard.Dynamics.Contacts {
  public struct ContactRegister {
    public ContactCreateFcn  CreateFcn;
    public ContactDestroyFcn DestroyFcn;
    public bool              Primary;
  }
}