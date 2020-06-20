using System.Numerics;
using Box2D.NetStandard.Collision;
using Box2D.NetStandard.Common;

namespace Box2D.NetStandard.Dynamics.Contacts {
  internal class ContactPositionConstraint {
    internal Vector2[]    localPoints = new Vector2[Settings.MaxManifoldPoints];
    internal Vector2      localNormal;
    internal Vector2      localPoint;
    internal int          indexA;
    internal int          indexB;
    internal float        invMassA;
    internal float        invMassB;
    internal Vector2      localCenterA;
    internal Vector2      localCenterB;
    internal float        invIA;
    internal float        invIB;
    internal ManifoldType type;
    internal float        radiusA;
    internal float        radiusB;
    internal int          pointCount;
  };
}