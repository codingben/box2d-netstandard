![Box2D Logo](https://box2d.org/images/logo.svg)

# Box2D .NET Standard 

Box2D .NET Standard is the port of Box2D from C++ to C# by Ben Ukhanov & Hugh Phoenix-Hulme 2020.

Loosely based on Box2DX by Ihar Kalasouski 2008.

- The purpose of this repository is to create Box2D .NET Standard library.
- The .NET Standard supports .NET Framework, .NET Core, and .NET 5.0.
- The NuGet package is available on [Box2D.NetStandard](https://www.nuget.org/packages/Box2D.NetStandard/).

## Current Status

- The latest non-pre-release version on NuGet (v1.x.x) is essentially Box2DX from 2008 updated to work with .NET Standard and cleaned up.
- The current pre-release version may be unstable. For example, an earlier version was known to produce Null Reference Exceptions. The API is unstable - the intent is to change all casing to lowerCamelCase to match Erin's code, and replace all Get...() and Set...(value) methods with Properties. Some features beyond Box2D 2.4.0 _are_ implemented, but not all.
If you take the pre-release version, PLEASE feel free to raise Issues and create Pull Requests, but at the same time DON'T expect perfection or stability.

## Known issues

In v1.0.x:
- None.

In v2.4.x:
- Code Optimisation must be enabled for things to work properly, and you **must not** have a debugger attached.

## Source

- [Ported C# Box2DX - Ihar Kalasouski](https://code.google.com/archive/p/box2dx/)
- [C++ Box2D Original - Erin Catto](https://github.com/erincatto/box2d)

## Contributing

Anyone who wants to contribute to this repository:
- The changes will be in the new branch (feature/new or feature/bug-fix).
- The new pull request will be started.
- The new version will be published.

## Features

### Collision

- Continuous collision detection.
- Contact callbacks: add, persist, remove.
- Convex polygons and circles.
- Multiple shapes per body.
- One-shot contact manifolds.
- Incremental sweep-and-prune broadphase.
- Efficient pair management.
- Fast broadphase AABB queries.
- Collision groups and categories.

### Physics

- Continuous physics with time of impact solver.
- Persistent body-joint-contact graph.
- Island solution and sleep management.
- Contact, friction, and restitution.
- Stable stacking with a linear-time solver.
- Revolute, prismatic, distance, pulley, gear, and mouse joints.
- Joint limits, motors, and friction.
- Momentum decoupled position correction.
- Fairly accurate reaction forces/impulses.

### System

- Centralized tuning parameters.
- Pure .NET Standard 2.0+ library.
- Please [See .NET Implementation Support](https://docs.microsoft.com/en-us/dotnet/standard/net-standard).

## Documentation

- [Manual](https://box2d.org/documentation/)
- [Reddit](https://www.reddit.com/r/box2d/)
- [Discord](https://discord.gg/NKYgCBP)

## License

Original C++ Box2D is developed by Erin Catto, under the [MIT license](https://en.wikipedia.org/wiki/MIT_License).
