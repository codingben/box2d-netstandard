/*
    Window Simulation Copyright © Ben Ukhanov 2020
*/

using System;
using System.Collections.Concurrent;
using Box2DX.Common;
using OpenTK;
using OpenTK.Graphics.OpenGL;
using OpenTK.Graphics;
using OpenTK.Input;
using Math = System.Math;
using Color = Box2DX.Dynamics.Color;

namespace HelloWorld
{
    public class SimulationWindow : GameWindow, IWindow
    {
        private readonly string windowTitle;
        private readonly ConcurrentQueue<Action> drawActions;

        private IView view;

        public SimulationWindow(string title, int width, int height)
            : base(width, height, GraphicsMode.Default, title, GameWindowFlags.FixedWindow)
        {
            windowTitle = title;
            drawActions = new ConcurrentQueue<Action>();
        }

        public void SetView(IView view)
        {
            this.view = view;
        }

        protected override void OnMouseWheel(MouseWheelEventArgs eventArgs)
        {
            base.OnMouseWheel(eventArgs);

            var value = (float)eventArgs.Value / 1000;
            if (value < Constants.MinimumCameraZoom)
            {
                value = Constants.MinimumCameraZoom;
            }

            if (view != null)
            {
                view.Zoom = value;
            }
        }

        protected override void OnKeyDown(KeyboardKeyEventArgs eventArgs)
        {
            base.OnKeyDown(eventArgs);

            if (eventArgs.Key == Key.Enter)
            {
                if (view != null)
                {
                    view.Position = Vector2.Zero;
                }
            }
        }

        protected override void OnUpdateFrame(FrameEventArgs eventArgs)
        {
            base.OnUpdateFrame(eventArgs);

            Title = $"{windowTitle} - FPS: {RenderFrequency:0.0}";

            if (Focused)
            {
                if (Mouse.GetState().IsButtonDown(MouseButton.Right))
                {
                    if (view != null)
                    {
                        var x = Mouse.GetState().X;
                        var y = Mouse.GetState().Y;
                        var direction = new Vector2(x, -y) - view.Position;

                        view.Position += direction * Constants.MouseMoveSpeed;
                    }
                }
                else
                {
                    if (view != null)
                    {
                        var x = GetHorizontal();
                        var y = GetVertical();
                        var direction = new Vector2(x, y);

                        view.Position += direction * Constants.KeyboardMoveSpeed;
                    }
                }
            }
        }

        protected override void OnRenderFrame(FrameEventArgs eventArgs)
        {
            base.OnRenderFrame(eventArgs);

            GL.Clear(ClearBufferMask.ColorBufferBit);
            GL.ClearColor(OpenTK.Color.CornflowerBlue);

            view?.Update();

            while (drawActions.TryDequeue(out var action))
            {
                action.Invoke();
            }

            SwapBuffers();
        }

        public float GetHorizontal()
        {
            var horizontal = 0;

            if (Keyboard.GetState().IsKeyDown(Key.Left))
            {
                horizontal = -1;
            }

            if (Keyboard.GetState().IsKeyDown(Key.Right))
            {
                horizontal = 1;
            }

            return horizontal;
        }

        public float GetVertical()
        {
            var vertical = 0;

            if (Keyboard.GetState().IsKeyDown(Key.Up))
            {
                vertical = 1;
            }

            if (Keyboard.GetState().IsKeyDown(Key.Down))
            {
                vertical = -1;
            }

            return vertical;
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
                    var v = center + (radius * new Vec2((float)Math.Cos(theta), (float)Math.Sin(theta)));

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
                    var v = center + (radius * new Vec2((float)Math.Cos(theta), (float)Math.Sin(theta)));

                    GL.Vertex2(v.X, v.Y);

                    theta += kIncrement;
                }

                GL.End();

                DrawSegment(center, center + (radius * axis), color);
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

                var p2 = p1 + (K_AXIS_SCALE * xf.R.Col1);

                GL.Vertex2(p2.X, p2.Y);
                GL.Color3(0.0f, 1.0f, 0.0f);
                GL.Vertex2(p1.X, p1.Y);

                p2 = p1 + (K_AXIS_SCALE * xf.R.Col2);

                GL.Vertex2(p2.X, p2.Y);
                GL.End();
            });
        }
    }
}