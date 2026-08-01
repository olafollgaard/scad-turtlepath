# `function TurtlePath3d()`

The main turtle function, that you provide with a number of steps for the 'turtle' to take.

## Parameters

- `steps` - A list of step instructions, each in the form of a `function(acc,dp)`

	- `acc` - The accumulated result so far, on the same format as the main result
	- `dp` - Debugging prefix for assertion messages

	Each step function must return its contribution to the main result, in the same form as the main result, with only the `path_pts` part being mandatory: The hv/nv vector lists, if omitted, will be automatically padded with the previous hv/nv values.

	Step instructions typically each generate a number of steps, e.g. to make a pseudo-rounded curve with `tp3d_Turn()`. The number of steps each step instruction generates is typically given as a parameter `n` to step utility functions like `tp3d_Turn(..., n=10)`. Use higher numbers to improve smoothness, lower numbers to improve performance. The two are, of course, inversely proportional.

	The 'horizontal' plane of the turtle is given by the `hv` ('forward') and `nv` ('up') vectors of the previous step.

- `start_pt` - Initial 3d position
	> default: `[0,0,0]`
- `start_hv` - Initial heading vector ('forward') in the 'horizontal' plane
	> default: `[0,1,0]`
- `start_nv` - Initial normal vector ('up') for the 'horizontal' plane
	> default: `[0,0,1]`
- `drop_last` - Drop last point in path, to avoid overlapping points when creating a loop
	> default: false
- `debug_prefix` - Prefix for assertion messages
	> has a default, but be kind to yourself and include a short but descriptive prefix
- `incl_debug_labels` - Automatically add a start label for each step
	> default: false

## Result

A list of intermediate results: `[ path_pts, path_hvs, path_nvs, labels, stored ]`

- `path_pts` - A 3d point for each step
- `path_hvs` - A 3d heading vector for each step
- `path_nvs` - A 3d 'horizontal' plane normal vector ('up') for each step
- `labels` - A `["label",<index>]` pair for each labeled step index, added with e.g.
	> `function(acc,dp) tp3d_AddLabel(acc,dp,"name")`
- `stored` - A `["name",<value>]` pair for each stored custom value, added with e.g.
	> `function(acc,dp) tp3d_AddStored(acc,dp,"name",[42,37])`

`labels` and `stored` can be used both in later step instructions and in alignment of multiple solids.

## Sample

```scad
TurtlePath3d(debug_prefix="sample path",steps=[
	// step forward 10 mm
	function(acc,dp) tp3d_Straight(acc,dp,d=10),
	// turn 45 degrees to the left with a 10 mm radius in 4 steps
	function(acc,dp) tp3d_Turn(acc,dp,a=45,r=10,n=4),
	// step forward another 10 mm
	function(acc,dp) tp3d_Straight(acc,dp,d=10),
])
```
