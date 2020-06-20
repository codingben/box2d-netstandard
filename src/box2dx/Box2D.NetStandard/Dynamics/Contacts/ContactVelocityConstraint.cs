using System.Numerics;
using Box2DX.Common;

namespace Box2DX.Dynamics {
  public class ContactVelocityConstraint {
    internal VelocityConstraintPoint[] points = new VelocityConstraintPoint[Settings.MaxManifoldPoints];
    internal Vector2                   normal;
    internal Mat22                     normalMass;
    internal Mat22                     K;
    internal int                       indexA;
    internal int                       indexB;
    internal float                     invMassA;
    internal float                     invMassB;
    internal float                     invIA;
    internal float                     invIB;
    internal float                     friction;
    internal float                     restitution;
    internal float                     tangentSpeed;
    internal int                       pointCount;
    internal int                       contactIndex;
  };
}