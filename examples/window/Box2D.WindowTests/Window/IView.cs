/*
    Window Simulation Copyright © Ben Ukhanov 2020
*/

using System.Numerics;

namespace Box2D.Window
{
    public interface IView
    {
        Vector2 Position { get; set; }

        float Zoom { get; set; }

        void Update();
    }
}