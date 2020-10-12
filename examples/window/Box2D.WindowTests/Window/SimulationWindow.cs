/*
    Window Simulation Copyright © Ben Ukhanov 2020
*/

using System;
using System.Collections.Concurrent;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using OpenTK.Graphics.OpenGL;
using Math = System.Math;
using Color = Box2D.NetStandard.Dynamics.World.Color;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.GraphicsLibraryFramework;
using System.Numerics;

namespace Box2D.Window
{
    public class SimulationWindow : GameWindow, IWindow
    {
        private readonly string windowTitle;
        private readonly Body focusBody;
        private readonly ConcurrentQueue<Action> drawActions;

        private IView view;

        public SimulationWindow(string title, int width, int height, Body focusBody = null)
            : base(
                new GameWindowSettings()
                {
                    RenderFrequency = 60.0,
                    UpdateFrequency = 60.0
                },
                new NativeWindowSettings()
                {
                    Title = title,
                    Size = new OpenTK.Mathematics.Vector2i(width, height)
                })
        {
            this.focusBody = focusBody;

            windowTitle = title;
            drawActions = new ConcurrentQueue<Action>();
        }

        public void SetView(IView view)
        {
            this.view = view;
        }

        public static bool stepNext;
        public static bool paused = false;

        private float mouseLast = 0;
        private int frames = 0;

        protected override void OnMouseWheel(MouseWheelEventArgs eventArgs)
        {
            base.OnMouseWheel(eventArgs);

            float mouseDelta = eventArgs.OffsetX - mouseLast;
            mouseLast = eventArgs.OffsetX;

            if (mouseDelta > 0) view.Zoom *= 1.2f;
            if (mouseDelta < 0) view.Zoom /= 1.2f;
        }

        protected override void OnKeyDown(KeyboardKeyEventArgs eventArgs)
        {
            base.OnKeyDown(eventArgs);

            if (eventArgs.Key == Keys.Enter)
            {
                if (view != null)
                {
                    view.Position = Vector2.Zero;
                }
            }

            if (eventArgs.Key == Keys.Space)
            {
                stepNext = true;
            }

            if (eventArgs.Key == Keys.P)
            {
                paused = !paused;
            }
        }

        protected override void OnUpdateFrame(FrameEventArgs eventArgs)
        {
            base.OnUpdateFrame(eventArgs);

            Title = $"{windowTitle} - FPS: {RenderFrequency:0.0} - Frames: {frames}";

            if (IsFocused)
            {
                if (IsMouseButtonDown(MouseButton.Right))
                {
                    if (view != null)
                    {
                        var x = MousePosition.X;
                        var y = MousePosition.Y;
                        var direction = new Vector2(x, -y) - view.Position;

                        view.Position += direction * WindowSettings.MouseMoveSpeed;
                    }
                }
                else
                {
                    if (view != null)
                    {
                        var x = GetHorizontal();
                        var y = GetVertical();
                        var direction = new Vector2(x, y);

                        view.Position += direction * WindowSettings.KeyboardMoveSpeed;
                    }
                }
            }
        }

        protected override void OnRenderFrame(FrameEventArgs eventArgs)
        {
            if (!paused) frames++;

            base.OnRenderFrame(eventArgs);

            GL.Clear(ClearBufferMask.ColorBufferBit);
            GL.ClearColor(OpenTK.Mathematics.Color4.Black);

            if (focusBody != null)
            {
                var bodyPosition = focusBody.GetPosition();
                view.Position = new Vector2(bodyPosition.X, bodyPosition.Y);
            }

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

            if (IsKeyDown(Keys.Left))
            {
                horizontal = -1;
            }

            if (IsKeyDown(Keys.Right))
            {
                horizontal = 1;
            }

            return horizontal;
        }

        public float GetVertical()
        {
            var vertical = 0;

            if (IsKeyDown(Keys.Up))
            {
                vertical = 1;
            }

            if (IsKeyDown(Keys.Down))
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
                    var vertex = vertices[i];

                    GL.Vertex2(vertex.X, vertex.Y);
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
                    var vertex = vertices[i];

                    GL.Vertex2(vertex.X, vertex.Y);
                }

                GL.End();
            });
        }

        public void DrawCircle(Vec2 center, float radius, Color color)
        {
            drawActions.Enqueue(() =>
            {
                const float kSegments = 16.0f;
                const int VertexCount = 16;

                var kIncrement = Settings.Tau / kSegments;
                var theta = 0.0f;

                GL.Color4(color.R, color.G, color.B, 0.5f);
                GL.Begin(PrimitiveType.LineLoop);
                GL.VertexPointer(VertexCount * 2, VertexPointerType.Float, 0, IntPtr.Zero);

                for (var i = 0; i < kSegments; ++i)
                {
                    var x = (float)Math.Cos(theta);
                    var y = (float)Math.Sin(theta);
                    var vertex = center + (radius * new Vec2(x, y));

                    GL.Vertex2(vertex.X, vertex.Y);

                    theta += kIncrement;
                }

                GL.End();
            });
        }

        public void DrawSolidCircle(Vec2 center, float radius, Vec2 axis, Color color)
        {
            drawActions.Enqueue(() =>
            {
                const float kSegments = 16.0f;
                const int VertexCount = 16;

                var kIncrement = Settings.Tau / kSegments;
                var theta = 0.0f;

                GL.Color4(color.R, color.G, color.B, 0.5f);
                GL.Begin(PrimitiveType.TriangleFan);
                GL.VertexPointer(VertexCount * 2, VertexPointerType.Float, 0, IntPtr.Zero);

                for (var i = 0; i < kSegments; ++i)
                {
                    var x = (float)Math.Cos(theta);
                    var y = (float)Math.Sin(theta);
                    var vertex = center + (radius * new Vec2(x, y));

                    GL.Vertex2(vertex.X, vertex.Y);

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

        public void DrawXForm(Transform xf)
        {
            drawActions.Enqueue(() =>
            {
                const float kAxisScale = 0.4f;

                var a = xf.p;

                GL.Begin(PrimitiveType.Lines);
                GL.Color3(1.0f, 0.0f, 0.0f);
                GL.Vertex2(a.X, a.Y);

                var ex = new System.Numerics.Vector2(xf.q.M11, xf.q.M21);

                var b = a + (kAxisScale * ex);

                GL.Vertex2(b.X, b.Y);
                GL.Color3(0.0f, 1.0f, 0.0f);
                GL.Vertex2(a.X, a.Y);

                var ey = new System.Numerics.Vector2(xf.q.M12, xf.q.M22);

                b = a + (kAxisScale * ey);

                GL.Vertex2(b.X, b.Y);
                GL.End();
            });
        }
    }
}
