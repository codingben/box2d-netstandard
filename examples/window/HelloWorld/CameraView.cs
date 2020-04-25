using OpenTK;
using OpenTK.Graphics.OpenGL;

namespace Physics.Box2D.PhysicsSimulation.Window
{
    public class CameraView
    {
        public const float MINIMUM_CAMERA_ZOON_VALUE = 0.001f;

        public Vector2 Position = Vector2.Zero;
        public float Zoom = MINIMUM_CAMERA_ZOON_VALUE;

        public void Update()
        {
            GL.LoadIdentity();

            var transform = Matrix4.Identity;
            transform = Matrix4.Mult(transform, Matrix4.CreateTranslation(-Position.X, -Position.Y, 0));
            transform = Matrix4.Mult(transform, Matrix4.CreateScale(Zoom, Zoom, 1.0f));

            GL.MultMatrix(ref transform);
        }

        public void Reset()
        {
            Position = Vector2.Zero;
        }
    }
}