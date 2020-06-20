using System;
using System.Net.Sockets;
using System.Runtime.InteropServices.ComTypes;
using Box2DX.Collision;
using int32 = System.Int32;
using b2Vec2 = System.Numerics.Vector2;

namespace Box2DX.Common {
  internal enum PointState {
    Null, Add, Persist, Remove
  }
  
  internal class Global {
    internal static void GetPointStates(PointState[] state1, PointState[] state2, in Manifold manifold1,
      in Manifold                             manifold2) {
      for (int32 i = 0; i < Settings.MaxManifoldPoints; ++i) {
        state1[i] = PointState.Null;
        state2[i] = PointState.Null;
      }

      // Detect persists and removes.
      for (int32 i = 0; i < manifold1.pointCount; ++i) {
        ContactID id = manifold1.points[i].id;

        state1[i] = PointState.Remove;

        for (int32 j = 0; j < manifold2.pointCount; ++j) {
          if (manifold2.points[j].id.key == id.key) {
            state1[i] = PointState.Persist;
            break;
          }
        }
      }

      // Detect persists and adds.
      for (int32 i = 0; i < manifold2.pointCount; ++i) {
        ContactID id = manifold2.points[i].id;

        state2[i] = PointState.Add;

        for (int32 j = 0; j < manifold1.pointCount; ++j) {
          if (manifold1.points[j].id.key == id.key) {
            state2[i] = PointState.Persist;
            break;
          }
        }
      }
    }
  }
}