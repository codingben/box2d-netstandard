/*
    Window Simulation Copyright © Ben Ukhanov 2020
*/

using OpenTK;

namespace HelloWorld
{
    public interface IView
    {
        Vector2 Position { get; set; }

        float Zoom { get; set; }

        void Update();
    }
}