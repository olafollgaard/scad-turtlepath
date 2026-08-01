# TurtlePath3d usage

This library consists of a number of `function`s that return intermediate lists, which in the end can be resolved into lists of points and faces for a `polyhedron()`. The one exception is `module TurtlePath3d_Sweep()`, which is more or less a shortcut to do this with some common ways of building faces from the generated points.

## `function TurtlePath3d()`

The main turtle function which you provide with a number of steps for the 'turtle' to take.

See [USAGE.TurtlePath3d.md](USAGE.TurtlePath3d.md)

## `module TurtlePath3d_Sweep()`

Shortcut for some common ways to make a solid from the points generated with a path and profile.

See [USAGE.TurtlePath3d_Sweep.md](USAGE.TurtlePath3d_Sweep.md)

## Path step utility functions

See [USAGE.StepUtilities.md](USAGE.StepUtilities.md) for a list of functions that can be used from within each `function(acc,dp)` in the `steps` parameter of `TurtlePath3d()`.

## Points and faces utility functions

If `TurtlePath3d_Sweep()` is insufficient to your use case, see [USAGE.SweepUtilities.md](USAGE.SweepUtilities.md) for a list of functions to help generate points and faces for a `polyhedron()`.

## Other utility functions

### `tp3d_GetPathXY(acc,skip_last=false)`

Generate a 2d path from `acc`, the accumulated/main result.

- Simply copies the X and Y parts of `path_pts` and drops the Z part
- Intended for generating a 2d profile for e.g. `TurtlePath3d_Sweep()` using `TurtlePath3d()`

### `tp3d_GetCursor(acc,debug_prefix="",step=-1)`

Extract `[pt,hv,nv]` from a step in the accumulated result.

- `step` - Use `>=0` to index from the start, `<0` to index 'backwards' from the end

### `tp3d_IdxByLabel(label,acc,debug_prefix="")`

Lookup the step index of the given label.

### `tp3d_StoredByName(name,acc,debug_prefix="")`

Lookup the stored value with the given name.

### `tp3d_DistToPlane(sp,hv,pp,pnv,debug_prefix="")`

Calculate distance from `sp` along `hv` to intersection with plane.

- `sp` - Starting point
- `hv` - Heading vector
- `pp` - Point on plane
- `pnv` - Plane normal vector

### `tp3d_CosFade(pct,a=0,b=1)`

'Crossfade', except using cosine instead of a flat interpolation, between `a` and `b`.

- `pct` - Percentage value, restricted to `>=0` and `<=1`
- The interpolation expression is `a + (b-a) * (1-cos(pct*180)) / 2`
