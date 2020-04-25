using System;
using System.Collections.Concurrent;
using System.ComponentModel;
using Box2DX.Common;
using Box2DX.Dynamics;
using CommonTools.Log;
using OpenTK;
using OpenTK.Graphics.OpenGL;
using OpenTK.Graphics;
using OpenTK.Input;
using Physics.Box2D.Components.Interfaces;
using IComponent = ComponentModel.Common.IComponent;
using IContainer = ComponentModel.Common.IContainer;
using Math = System.Math;

namespace HelloWorld
{
    public class PhysicsSimulationWindow : GameWindow, IComponent, IPhysicsSimulationWindow
    {
        private const float CAMERA_MOVEMENT_SPEED = 0.001f;
        private const float MOVE_SPEED_VIA_KEYBOARD_MINIMUM_VALUE = 1;

        private float moveSpeedViaKeyboard = 1;

        private readonly string windowTitle;
        private readonly CameraView cameraView = new CameraView();

        private Action removeDebugDrawer;
        private readonly ConcurrentQueue<Action> drawActions = new ConcurrentQueue<Action>();

        public PhysicsSimulationWindow(string title, int width, int height)
            : base(width, height, GraphicsMode.Default, title, GameWindowFlags.FixedWindow)
        {
            windowTitle = title;
        }

        public void Awake(IContainer components)
        {
            var drawPhysics = new DrawPhysics(this);
            drawPhysics.AppendFlags(DebugDraw.DrawFlags.Aabb);
            drawPhysics.AppendFlags(DebugDraw.DrawFlags.Shape);

            var physicsWorld = components.GetComponent<IPhysicsWorldProvider>().AssertNotNull();
            var world = physicsWorld.GetWorld();
            world.SetDebugDraw(drawPhysics);

            removeDebugDrawer = () => world?.SetDebugDraw(null);
        }

        private void MoveCameraViewViaMouse()
        {
            if (!IsMouseKeyDown() || !Focused)
            {
                return;
            }

            var direction = GetMousePosition() - cameraView.Position;
            cameraView.Position += direction * CAMERA_MOVEMENT_SPEED;
        }

        private void MoveCameraViewViaKeyboard()
        {
            if (!Focused || IsMouseKeyDown())
            {
                return;
            }

            if (Keyboard.GetState().IsKeyDown(Key.Left))
            {
                cameraView.Position += new Vector2(-1, 0) * moveSpeedViaKeyboard;
            }

            if (Keyboard.GetState().IsKeyDown(Key.Right))
            {
                cameraView.Position += new Vector2(1, 0) * moveSpeedViaKeyboard;
            }

            if (Keyboard.GetState().IsKeyDown(Key.Up))
            {
                cameraView.Position += new Vector2(0, 1) * moveSpeedViaKeyboard;
            }

            if (Keyboard.GetState().IsKeyDown(Key.Down))
            {
                cameraView.Position += new Vector2(0, -1) * moveSpeedViaKeyboard;
            }
        }

        protected override void OnMouseWheel(MouseWheelEventArgs e)
        {
            base.OnMouseWheel(e);

            var value = (float)e.Value / 1000;
            if (value < CameraView.MINIMUM_CAMERA_ZOON_VALUE)
            {
                value = CameraView.MINIMUM_CAMERA_ZOON_VALUE;
            }

            cameraView.Zoom = value;
        }

        protected override void OnKeyDown(KeyboardKeyEventArgs e)
        {
            base.OnKeyDown(e);

            if (e.Key == Key.KeypadPlus)
            {
                moveSpeedViaKeyboard += MOVE_SPEED_VIA_KEYBOARD_MINIMUM_VALUE;
            }

            if (e.Key == Key.KeypadMinus)
            {
                if (Math.Abs(moveSpeedViaKeyboard) <= MOVE_SPEED_VIA_KEYBOARD_MINIMUM_VALUE)
                {
                    moveSpeedViaKeyboard = MOVE_SPEED_VIA_KEYBOARD_MINIMUM_VALUE;
                    return;
                }

                moveSpeedViaKeyboard -= MOVE_SPEED_VIA_KEYBOARD_MINIMUM_VALUE;
            }

            if (e.Key == Key.Enter)
            {
                cameraView.Reset();
            }
        }

        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            base.OnUpdateFrame(e);

            Title = $"{windowTitle} - FPS: {RenderFrequency:0.0} - Move Speed Via Keyboard: {moveSpeedViaKeyboard}";

            MoveCameraViewViaMouse();
            MoveCameraViewViaKeyboard();
        }

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            base.OnRenderFrame(e);

            GL.Clear(ClearBufferMask.ColorBufferBit);
            GL.ClearColor(System.Drawing.Color.CornflowerBlue);

            cameraView.Update();

            while (drawActions.TryDequeue(out var action))
            {
                action.Invoke();
            }

            SwapBuffers();
        }

        protected override void OnClosing(CancelEventArgs e)
        {
            base.OnClosing(e);

            removeDebugDrawer?.Invoke();
        }

        private bool IsMouseKeyDown()
        {
            const MouseButton CAMERA_MOVEMENT_MOUSE_KEY = MouseButton.Right;
            return OpenTK.Input.Mouse.GetState().IsButtonDown(CAMERA_MOVEMENT_MOUSE_KEY);
        }

        private Vector2 GetMousePosition()
        {
            return new Vector2(OpenTK.Input.Mouse.GetState().X, -OpenTK.Input.Mouse.GetState().Y);
        }

        public void DrawPolygon(Vec2[] vertices, int vertexCount, Color color)
        {
            drawActions.Enqueue(() =>
            {
                GL.Color4(color.R, color.G, color.B, 0.5f);
                GL.Begin(PrimitiveType.LineLoop);

                for (var i = 0; i < vertexCount; i++)
                {
                    var v = vertices[i];
                    GL.Vertex2(v.X, v.Y);
                }

                GL.End();
            });
        }

        public void DrawSolidPolygon(Vec2[] vertices, int vertexCount, Color color)
        {
            drawActions.Enqueue(() =>
            {
                GL.Color4(color.R, color.G, color.B, 0.5f);
                GL.Begin(PrimitiveType.TriangleFan);

                for (var i = 0; i < vertexCount; i++)
                {
                    var v = vertices[i];
                    GL.Vertex2(v.X, v.Y);
                }

                GL.End();
            });
        }

        public void DrawCircle(Vec2 center, float radius, Color color)
        {
            drawActions.Enqueue(() =>
            {
                const float K_SEGMENTS = 16.0f;
                const int VERTEX_COUNT = 16;
                var kIncrement = 2.0f * Settings.Pi / K_SEGMENTS;
                var theta = 0.0f;

                GL.Color4(color.R, color.G, color.B, 0.5f);
                GL.Begin(PrimitiveType.LineLoop);

                GL.VertexPointer(VERTEX_COUNT * 2, VertexPointerType.Float, 0, IntPtr.Zero);

                for (var i = 0; i < K_SEGMENTS; ++i)
                {
                    var v = center + radius * new Vec2((float)Math.Cos(theta), (float)Math.Sin(theta));
                    GL.Vertex2(v.X, v.Y);
                    theta += kIncrement;
                }

                GL.End();
            });
        }

        public void DrawSolidCircle(Vec2 center, float radius, Vec2 axis, Color color)
        {
            drawActions.Enqueue(() =>
            {
                const float K_SEGMENTS = 16.0f;
                const int VERTEX_COUNT = 16;
                var kIncrement = 2.0f * Settings.Pi / K_SEGMENTS;
                var theta = 0.0f;

                GL.Color4(color.R, color.G, color.B, 0.5f);
                GL.Begin(PrimitiveType.TriangleFan);

                GL.VertexPointer(VERTEX_COUNT * 2, VertexPointerType.Float, 0, IntPtr.Zero);

                for (var i = 0; i < K_SEGMENTS; ++i)
                {
                    var v = center + radius * new Vec2((float)Math.Cos(theta), (float)Math.Sin(theta));
                    GL.Vertex2(v.X, v.Y);
                    theta += kIncrement;
                }

                GL.End();

                DrawSegment(center, center + radius * axis, color);
            });
        }

        public void DrawSegment(Vec2 p1, Vec2 p2, Color color)
        {
            drawActions.Enqueue(() =>
            {
                GL.Color4(color.R, color.G, color.B, 1);
                GL.Begin(PrimitiveType.Lines);
                GL.Vertex2(p1.X, p1.Y);
                GL.Vertex2(p2.X, p2.Y);
                GL.End();
            });
        }

        public void DrawXForm(XForm xf)
        {
            drawActions.Enqueue(() =>
            {
                const float K_AXIS_SCALE = 0.4f;
                var p1 = xf.Position;

                GL.Begin(PrimitiveType.Lines);
                GL.Color3(1.0f, 0.0f, 0.0f);
                GL.Vertex2(p1.X, p1.Y);

                var p2 = p1 + K_AXIS_SCALE * xf.R.Col1;

                GL.Vertex2(p2.X, p2.Y);
                GL.Color3(0.0f, 1.0f, 0.0f);
                GL.Vertex2(p1.X, p1.Y);

                p2 = p1 + K_AXIS_SCALE * xf.R.Col2;

                GL.Vertex2(p2.X, p2.Y);
                GL.End();
            });
        }
    }
}