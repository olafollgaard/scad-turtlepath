# TurtlePath3d

The intent in writing this library is to make it easier to create more complicated shapes in OpenSCAD, especially rounded or curved shapes, without having to manually manage and align centers and radii in a curved path. It started with wanting to make it easier to create profiles for `linear_extrude`/`rotate_extrude`, then quickly escalated to writing a full `sweep`-type library.

It is not intended for organic shapes, though some curved results might look almost organic while being strictly parameterized. The closest it comes to organic shapes is a basic cubic [Bezier curve](https://en.wikipedia.org/wiki/B%C3%A9zier_curve) function, but I think that would be a rough job. IMHO, OpenSCAD is not suited for that kind of job anyway.

- Named in homage to the old-school but easy to learn [Turtle Graphics](https://en.wikipedia.org/wiki/Turtle_graphics), which strongly inspired the mode of usage
- Native 3d, supporting Yaw/Turn (left/right), Pitch (up/down) and Roll (clockwise/counterclockwise) operations
- Primary angle and distance parameters are relative to the current position and orientation, with some absolute value parameters thrown in when convenient
- Contains lots of assertions, in an attempt to avoid the tendency of OpenSCAD to spend an unreasonable amount of uninterruptible time and processing power creating lots and lots of warnings about undefined values, instead of telling you up front that you have spelled a variable name wrong.

## License

This library is published under the [MIT license](LICENSE)

## Usage

See [USAGE.md](USAGE.md).

## OpenSCAD version

As of time of writing, I am using OpenSCAD version 2026.07.01.fp (git 34a81df5b).

I do not know if this library will work with the official latest release, which is version 2021.01.
