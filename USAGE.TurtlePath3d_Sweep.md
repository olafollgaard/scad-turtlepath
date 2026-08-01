# `module TurtlePath3d_Sweep()`

- Generates points via a given profile and path
- Generates faces in several common ways
- Then makes a solid via `polyhedron()` with those points and faces

## Parameters

* `profile` - The profile to sweep along the path
	- Either a simple list of 2d points, e.g. `[ [-2,+1], [0,+2], [+2,+1], [+1,-2], [-1,-2] ]`
	- Or a `function(i)` (with `i` being the step index) which must generate a simple list of 2d points.
		> [!NOTE]
		>
		> Results for all values of `i` must be of the same length.

		This is where the advanced (as in mind-bending) fun begins: change the profile depending on the position in the path.

* `path` - The result of a call to `TurtlePath3d()`
* `closure=["bar"]` - How the faces should be generated:
	* `["bar"]` or `["face-ends"]`
		- loop the profile points
		- use points from `path.x[0]` as edge of start face
		- use points from `path.x[n-1]` as edge of end face
		- add face points together, for OpenSCAD to sort out triangles
	* `["zip-ends",<startidx=0>]`
		- like `["face-ends"]`, except for face triangles:
		- 'zip' face points, starting from 0 or a given profile index
	* `["torus"]` or `["path-loop"]`
		- loop both profile points and path points
		- there are no end faces, result will be a torus
	* `["plate-edge"]` or `["face-edges"]`
		- loop the path points, but not the profile points
		- use generated points from `profile[0]` along path as edge of 'start' face
		- use generated points from `profile[n-1]` along path as edge of 'end' face
		- add face points together, for OpenSCAD to sort out triangles
	* `["zip-edges"]`
		- like `["face-edges"]`, except for face triangles:
		- 'zip' face points
	* `["tube",<inner_profile>]`
		- use the given `inner_profile` to create the inside wall of the tube
		- `inner_profile` must be equivalent to the outer profile, i.e. it must be  either a alist of 2d points or a `function(i)` to generate one
		- `inner_profile` must have the same number of points as the outer one
		- profiles are looped and must be path-loop compatible
	* `["cap-tube",<inner_profile>,<cap-step-count=1>]`
		- like `tube`, but also:
		- use outer profile points from `path.x[0]` as edge of the outside cap face
		- use inner profile points from `path.x[<cap-step-count>]` as edge of the inside cap face
* `debug_prefix` - Prefix for assertion messages

> [!CAUTION]
>
> Sweeping a too-large profile through a tight corner can lead to invalid solids.
>
> A warning will appear in the OpenSCAD console for each step where the algorithm detects a possible too-sharp curve for the profile. This was an assertion until I encountered a situation with a dynamic profile where it was not *actually* too sharp a curve, but I kept it as a warning because it usually is, plus it is often easier to spot the position in the preview.

## Sample

```scad
TurtlePath3d_Sweep(
	debug_prefix="Coil",
	profile=[[0,0],[2,+1],[2,-1]],
	path=TurtlePath3d(debug_prefix="Coil path",start_pt=[2,0,0],steps=[
		function(acc,dp) tp3d_Turn(acc,dp,a=2*360,r=2,dn=8,n=30)
	])
);
```
