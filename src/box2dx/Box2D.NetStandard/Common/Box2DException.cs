using System;

namespace Box2D.NetStandard.Common
{
    public class Box2DException : Exception
    {
        public Box2DException(string message) : base(message)
        { }
    }
}