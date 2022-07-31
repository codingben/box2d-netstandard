using System.Numerics;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Common;

namespace Box2D.NetStandard.Collision
{
    internal class PolygonAndPolygonCollider : Collider<PolygonShape, PolygonShape>
    {
        internal override void Collide(
            out Manifold manifold,
            in PolygonShape polyA,
            in Transform xfA,
            in PolygonShape polyB,
            in Transform xfB)
        {
            manifold = new Manifold();
            //manifold.pointCount = 0;
            float totalRadius = polyA.m_radius + polyB.m_radius;

            float separationA = FindMaxSeparation(out int edgeA, polyA, xfA, polyB, xfB);
            if (separationA > totalRadius)
            {
                return;
            }

            float separationB = FindMaxSeparation(out int edgeB, polyB, xfB, polyA, xfA);
            if (separationB > totalRadius)
            {
                return;
            }

            PolygonShape poly1; // reference poly
            PolygonShape poly2; // incident poly
            Transform xf1, xf2;
            int edge1; // reference edge
            byte flip;
            const float k_tol = 0.1f * Settings.LinearSlop;

            if (separationB > separationA + k_tol)
            {
                poly1 = polyB;
                poly2 = polyA;
                xf1 = xfB;
                xf2 = xfA;
                edge1 = edgeB;
                manifold.type = ManifoldType.FaceB;
                flip = 1;
            }
            else
            {
                poly1 = polyA;
                poly2 = polyB;
                xf1 = xfA;
                xf2 = xfB;
                edge1 = edgeA;
                manifold.type = ManifoldType.FaceA;
                flip = 0;
            }

            FindIncidentEdge(out ClipVertex[] incidentEdge, poly1, xf1, edge1, poly2, xf2);

            int count1 = poly1.m_count;
            Vector2[] vertices1 = poly1.m_vertices;

            int iv1 = edge1;
            int iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;

            Vector2 v11 = vertices1[iv1];
            Vector2 v12 = vertices1[iv2];

            var localTangent = Vector2.Normalize(v12 - v11);

            Vector2 localNormal = Vectex.Cross(localTangent, 1.0f);
            Vector2 planePoint = 0.5f * (v11 + v12);

            var tangent = Vector2.Transform(localTangent, xf1.q); //Math.Mul(xf1.q, localTangent);
            Vector2 normal = Vectex.Cross(tangent, 1.0f);

            v11 = Math.Mul(xf1, v11);
            v12 = Math.Mul(xf1, v12);

            float frontOffset = Vector2.Dot(normal, v11);

            float sideOffset1 = -Vector2.Dot(tangent, v11) + totalRadius;
            float sideOffset2 = Vector2.Dot(tangent, v12) + totalRadius;

            // Clip incident edge against extruded edge1 side edges.
            int np;

            // Clip to box side 1
            np = Collision.ClipSegmentToLine(out ClipVertex[] clipPoints1, incidentEdge, -tangent, sideOffset1, iv1);

            if (np < 2)
            {
                return;
            }

            // Clip to negative box side 1
            np = Collision.ClipSegmentToLine(out ClipVertex[] clipPoints2, clipPoints1, tangent, sideOffset2, iv2);

            if (np < 2)
            {
                return;
            }

            // Now clipPoints2 contains the clipped points.
            manifold.localNormal = localNormal;
            manifold.localPoint = planePoint;

            var pointCount = 0;
            for (var i = 0; i < Settings.MaxManifoldPoints; ++i)
            {
                float separation = Vector2.Dot(normal, clipPoints2[i].v) - frontOffset;

                if (separation <= totalRadius)
                {
                    var cp = new ManifoldPoint();
                    cp.localPoint = Math.MulT(xf2, clipPoints2[i].v);
                    cp.id = clipPoints2[i].id;
                    if (flip != 0)
                    {
                        // Swap features
                        ContactFeature cf = cp.id.cf;
                        cp.id.cf.indexA = cf.indexB;
                        cp.id.cf.indexB = cf.indexA;
                        cp.id.cf.typeA = cf.typeB;
                        cp.id.cf.typeB = cf.typeA;
                    }

                    manifold.points[pointCount] = cp;
                    ++pointCount;
                }
            }

            manifold.pointCount = pointCount;
        }

        private static float FindMaxSeparation(out int edgeIndex, in PolygonShape poly1, in Transform xf1,
            in PolygonShape poly2, in Transform xf2)
        {
            int count1 = poly1.m_count;
            int count2 = poly2.m_count;
            Vector2[] n1s = poly1.m_normals;
            Vector2[] v1s = poly1.m_vertices;
            Vector2[] v2s = poly2.m_vertices;
            Transform xf = Math.MulT(xf2, xf1);

            var bestIndex = 0;
            float maxSeparation = float.MinValue;
            for (var i = 0; i < count1; ++i)
            {
                // Get poly1 normal in frame2.
                var n = Vector2.Transform(n1s[i], xf.q); // Math.Mul(xf.q, n1s[i]);
                Vector2 v1 = Math.Mul(xf, v1s[i]);

                // Find deepest point for normal i.
                float si = float.MaxValue;
                for (var j = 0; j < count2; ++j)
                {
                    float sij = Vector2.Dot(n, v2s[j] - v1);
                    if (sij < si)
                    {
                        si = sij;
                    }
                }

                if (si > maxSeparation)
                {
                    maxSeparation = si;
                    bestIndex = i;
                }
            }

            edgeIndex = bestIndex;
            return maxSeparation;
        }

        private static void FindIncidentEdge(
            out ClipVertex[] c,
            in PolygonShape poly1,
            in Transform xf1,
            int edge1,
            in PolygonShape poly2,
            in Transform xf2)
        {
            Vector2[] normals1 = poly1.m_normals;

            int count2 = poly2.m_count;
            Vector2[] vertices2 = poly2.m_vertices;
            Vector2[] normals2 = poly2.m_normals;

            //Debug.Assert(0 <= edge1 && edge1 < poly1.m_count);

            // Get the normal of the reference edge in poly2's frame.
            Vector2 normal1 =
                Math.MulT(xf2.q, Vector2.Transform(normals1[edge1], xf1.q)); // Math.Mul(xf1.q, normals1[edge1]));

            // Find the incident edge on poly2.
            var index = 0;
            float minDot = float.MaxValue;
            for (var i = 0; i < count2; ++i)
            {
                float dot = Vector2.Dot(normal1, normals2[i]);
                if (dot < minDot)
                {
                    minDot = dot;
                    index = i;
                }
            }

            // Build the clip vertices for the incident edge.
            int i1 = index;
            int i2 = i1 + 1 < count2 ? i1 + 1 : 0;
            c = new ClipVertex[2];
            c[0].v = Math.Mul(xf2, vertices2[i1]);
            c[0].id.cf.indexA = (byte)edge1;
            c[0].id.cf.indexB = (byte)i1;
            c[0].id.cf.typeA = (byte)ContactFeatureType.Face;
            c[0].id.cf.typeB = (byte)ContactFeatureType.Vertex;

            c[1].v = Math.Mul(xf2, vertices2[i2]);
            c[1].id.cf.indexA = (byte)edge1;
            c[1].id.cf.indexB = (byte)i2;
            c[1].id.cf.typeA = (byte)ContactFeatureType.Face;
            c[1].id.cf.typeB = (byte)ContactFeatureType.Vertex;
        }
    }
}