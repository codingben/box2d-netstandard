/*
 * Window Simulation Copyright © Ben Ukhanov 2021
 */

using System;
using System.Collections.Concurrent;
using Box2D.NetStandard.Common;
using Box2D.NetStandard.Dynamics.Bodies;
using OpenTK;
using OpenTK.Graphics.OpenGL;
using OpenTK.Graphics;
using OpenTK.Input;
using Math = System.Math;
using Color = Box2D.NetStandard.Dynamics.World.Color;

namespace Box2D.Window
{
    public class SimulationWindow : GameWindow, IWindow
    {
        public static bool StepNext;
        public static bool Paused;

        private readonly string title;
        private readonly Body focusBody;
        private readonly ConcurrentQueue<Action> drawActions;

        private IView view;

        private int mouseLast = 0;
        private int frames = 0;

        public SimulationWindow(string title, int width, int height, Body focusBody = null)
            : base(width, height, GraphicsMode.Default, title, GameWindowFlags.FixedWindow)
        {
            this.title = title;
            this.focusBody = focusBody;

            drawActions = new ConcurrentQueue<Action>();
        }

        public void SetView(IView view)
        {
            this.view = view;
        }

        protected override void OnMouseWheel(MouseWheelEventArgs eventArgs)
        {
            base.OnMouseWheel(eventArgs);

            var mouseDelta = eventArgs.Value - mouseLast;
            mouseLast = eventArgs.Value;

            if (mouseDelta > 0) view.Zoom *= 1.2f;
            if (mouseDelta < 0) view.Zoom /= 1.2f;
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

            if (eventArgs.Key == Key.Space)
            {
                StepNext = true;
            }

            if (eventArgs.Key == Key.P)
            {
                Paused = !Paused;
            }

            if (eventArgs.Key == Key.Minus || eventArgs.Key == Key.KeypadMinus)
            {
                view.Zoom /= 1.2f;
            }

            if (eventArgs.Key == Key.Plus || eventArgs.Key == Key.KeypadPlus)
            {
                view.Zoom *= 1.2f;
            }
        }

        protected override void OnUpdateFrame(FrameEventArgs eventArgs)
        {
            base.OnUpdateFrame(eventArgs);

            Title = $"{title} - FPS: {RenderFrequency:0.0} - Frames: {frames}";

            if (Focused)
            {
                if (Mouse.GetState().IsButtonDown(MouseButton.Right))
                {
                    if (view != null)
                    {
                        var x = Mouse.GetState().X;
                        var y = Mouse.GetState().Y;
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
            if (!Paused) frames++;

            base.OnRenderFrame(eventArgs);

            GL.Clear(ClearBufferMask.ColorBufferBit);
            GL.ClearColor(OpenTK.Color.Black);

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