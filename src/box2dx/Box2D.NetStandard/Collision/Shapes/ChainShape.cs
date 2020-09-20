using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using Box2D.NetStandard.Common;
using Math = Box2D.NetStandard.Common.Math;

namespace Box2D.NetStandard.Collision.Shapes
{
	/// <summary>
	///  /// The chain has one-sided collision, with the surface normal pointing to the right of the edge.
	///  This provides a counter-clockwise winding like the polygon shape.
	///  Connectivity information is used to create smooth collisions.
	/// </summary>
	/// <warning>The chain will not collide properly if there are self-intersections.</warning>
	public class ChainShape : Shape
	{
		internal const byte contactMatch = 3;
		internal int m_count;
		internal Vector2 m_prevVertex, m_nextVertex;
		internal Vector2[] m_vertices;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public ChainShape() => m_radius = Settings.PolygonRadius;

		public Vector2[] Vertices
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => m_vertices;
		}

		internal override byte ContactMatch => contactMatch;

		/// <summary>
		///  Create a loop. This automatically adjusts connectivity.
		/// </summary>
		/// <param name="vertices">An array of vertices. These are copied</param>
		public void CreateLoop(in Vector2[] vertices)
		{
			int count = vertices.Length;
			if (count < 3)
			{
				return;
			}

			m_count = count + 1;
			m_vertices = new Vector2[m_count];
			Array.Copy(vertices, m_vertices, count);
			m_vertices[count] = m_vertices[0];
			m_prevVertex = m_vertices[m_count - 2];
			m_nextVertex = m_vertices[1];
		}

		/// <summary>
		///  Create a chain with ghost vertices to connect multiple chains together.
		/// </summary>
		/// <param name="vertices">An array of vertices. These are copied</param>
		public void CreateChain(in Vector2[] vertices, in Vector2 prevVertex, in Vector2 nextVertex)
		{
			int count = vertices.Length;

			m_count = count;
			m_vertices = new Vector2[m_count];
			Array.Copy(vertices, m_vertices, m_count);

			m_prevVertex = prevVertex;
			m_nextVertex = nextVertex;
		}

		public override Shape Clone() => (ChainShape) MemberwiseClone();

		public override int GetChildCount() => m_count - 1;

		public void GetChildEdge(out EdgeShape edge, int index)
		{
			edge = new EdgeShape
			       {
				       m_radius = m_radius,
				       m_vertex0 = index > 0 ? m_vertices[index - 1] : m_prevVertex,
				       m_vertex1 = m_vertices[index + 0],
				       m_vertex2 = m_vertices[index + 1],
				       m_vertex3 = index < m_count - 2 ? m_vertices[index + 2] : m_nextVertex,
				       m_oneSided = true
			       };
		}

		public override bool TestPoint(in Transform xf, in Vector2 p) => false;

		public override bool RayCast(
			out RayCastOutput output,
			in RayCastInput input,
			in Transform transform,
			int childIndex)
		{
			var edgeShape = new EdgeShape();

			int i1 = childIndex;
			int i2 = childIndex + 1;
			if (i2 == m_count)
			{
				i2 = 0;
			}

			edgeShape.m_vertex1 = m_vertices[i1];
			edgeShape.m_vertex2 = m_vertices[i2];

			return edgeShape.RayCast(out output, input, transform, 0);
		}

		public override void ComputeAABB(out AABB aabb, in Transform xf, int childIndex)
		{
			int i1 = childIndex;
			int i2 = childIndex + 1;
			if (i2 == m_count)
			{
				i2 = 0;
			}

			Vector2 v1 = Math.Mul(xf, m_vertices[i1]);
			Vector2 v2 = Math.Mul(xf, m_vertices[i2]);

			aabb.lowerBound = Vector2.Min(v1, v2);
			aabb.upperBound = Vector2.Max(v1, v2);
		}

		public override void ComputeMass(out MassData massData, float density)
		{
			massData = default;
		}
	}
}