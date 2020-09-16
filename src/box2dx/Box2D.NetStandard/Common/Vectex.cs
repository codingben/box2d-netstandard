using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Box2D.NetStandard.Common
{
	public static class Vectex // vector extensions
	{
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static float GetIdx(in this Vector2 candidate, in int n) => n == 0 ? candidate.X : candidate.Y;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Vector2 Cross(float s, Vector2 a) => new Vector2(-s * a.Y, s * a.X);

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Vector2 Cross(Vector2 a, float s) => new Vector2(s * a.Y, -s * a.X);

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static float Cross(Vector2 a, Vector2 b) => a.X * b.Y - a.Y * b.X;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static bool IsValid(this Vector2 candidate) => Math.IsValid(candidate.X) && Math.IsValid(candidate.Y);

		[Obsolete("This is now a System.Numerics.Vector2, and cannot be mutated this way. Please create a new Vector2 and assign it to the property or field you're trying to modify.",
		          true)]
		public static void Set(this Vector2 v, float x, float y)
		{ }

		[Obsolete("This is now a System.Numerics.Vector2, and cannot be mutated this way. Please create a new Vector2 and assign it to the property or field you're trying to modify.",
		          true)]
		public static void Set(this Vector2 v, float x)
		{ }

		[Obsolete("This is now a System.Numerics.Vector2, and cannot be mutated this way. Please create a new Vector2 and assign it to the property or field you're trying to modify.",
		          true)]
		public static void SetZero(this Vector2 v)
		{ }
	}
}