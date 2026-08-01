# Point and face generation utility functions

If your shape can't be faced correctly with `TurtlePath3d_Sweep()`, you can use these utility functions to help generate points and faces for `polyhedron()`.

### `tp3d_SweepProfilePoints(path,debug_prefix,profile=function(step) [])`

- Generates points by sweeping along `path`
	- Uses `pt`, `hv`, `nv` of each step to rotate and translate the points provided by `profile`
	- `pt` is used as `[0,0]` in the profile plane
	- `hv` is used as a normal vector to the profile plane
	- `nv` is used as `[0,1]` in the profile plane
	- e.g. with `pt=[10,20,30], hv=[0,1,0], nv=[0,0,1]`, profile 2d point `[1,2]` will be rotated and translated into 3d point `[11,20,32]`
- `path` - The result of a call to `TurtlePath3d()`
- `debug_prefix` - Prefix for assertion messages
- `profile` - A `function(step)` that returns a simple list of 2d points
	- List must be the same length for all values of `step`

> [!CAUTION]
>
> Sweeping a too-large profile through a tight corner can lead to invalid solids.
>
> A warning will appear in the OpenSCAD console for each step where the algorithm detects a possible too-sharp curve for the profile. This was an assertion until I encountered a situation with a dynamic profile where it was not *actually* too sharp a curve, but I kept it as a warning because it usually is, plus it is often easier to spot the position in the preview.

### `tp3d_Sweep_GetEdgePt(path,profile_len,edge,pt,debug_prefix)`

- Calculate point index of `profile[0]` or `profile[n-1]` at position `pt` on the given `path`
- `path` - The result of a call to `TurtlePath3d()`
- `profile_len` - Profile length
- `edge` - Which edge: `"ps" => profile[0], "pe" => profile[profile_len-1]`
- `pt` - Index or label of position along path
	- If out of bounds, it wraps around the path like `path_loop==true`
- `debug_prefix` - Prefix for assertion messages

### `tp3d_Sweep_GetEdge(path,profile_len,edge,edge_s,edge_e,edge_d=0,debug_prefix)`

- Get edge point index list
- `path` - The result of a call to `TurtlePath3d()`
- `profile_len` - Profile length
- `edge` - Which edge: `"ps" => profile[0], "pe" => profile[profile_len-1]`
- `edge_s` and `edge_e` - Index or label of start/end positions along path
	- If out of bounds, it wraps around the path like `path_loop==true`
- `edge_d` - Direction (0=auto, 1=forward, -1=backward)
	- Necessary if you want the list to wrap around the path
- `debug_prefix` - Prefix for assertion messages

### `tp3d_Sweep_EdgeRibbonFaces(path,profile_len,l_edge_s,l_edge_e,r_edge_s,r_edge_e,l_edge_d,r_edge_d,edge="both",debug_prefix)`

- Build a ribbon of faces for `polyhedron()`, like using `closure=["zip-edges"]` in `TurtlePath3d_Sweep()`, but for a specific range of points along `path` instead of the whole length
- `path` - The result of a call to `TurtlePath3d()`
- `profile_len` - Profile length
- `l_edge_s,l_edge_e,l_edge_d` - Left edge start, end, direction (passed to `tp3d_GetEdge()`)
- `r_edge_s,r_edge_e,r_edge_d` - Right edge start, end, direction (passed to `tp3d_GetEdge()`)
- `edge` - Which edge: `"ps" => profile[0], "pe" => profile[profile_len-1], "both" => both`
- `debug_prefix` - Prefix for assertion messages

### `tp3d_Sweep_RibbonFaces(L,R,debug_prefix)`

- Build a ribbon of faces for `polyhedron()` by specifying the point index list of the left and the right side of the ribbon
- `L` and `R` - Left and right side
- `debug_prefix` - Prefix for assertion messages

### `tp3d_Sweep_EdgeTileFaces(path,profile_len,l_edge_s,l_edge_e,r_edge_s,r_edge_e,edge="both",debug_prefix)`

- Build tile faces from edge points
- `path` - The result of a call to `TurtlePath3d()`, only used for path length (`len(path.x)`) and labels (`path[3]`)
- `profile_len` - Profile length
- `l_edge_s` and `l_edge_e` - Start and end index/label of the left side of the tile
- `r_edge_s` and `r_edge_e` - Start and end index/label of the right side of the tile
- `edge` - Which edge: `"ps" => profile[0], "pe" => profile[profile_len-1], "both" => both`
- `debug_prefix` - Prefix for assertion messages

### `tp3d_Sweep_WallFaces(profile_len,path_len,path_loop=false,profile_loop=false,debug_prefix)`

- Generate wall faces for profile swept along a path
- `profile_len` - Profile length
- `path_len` - Path length
- `path_loop` - Closed loop path
	- Bridge the gap between `path[0]` and `path[path_len-1]`
	- If false, those gaps will not be filled by this function
- `profile_loop` - Closed loop profile
	- Bridge the gap between `profile[0]` and `profile[profile_len-1]`
	- If false, those gaps will not be filled by this function
- `debug_prefix` - Prefix for assertion messages

### `tp3d_Sweep_EndFaces(ptn,prn,s=true,e=true)`

- Build closure faces by letting OpenSCAD sort out triangles for the end faces
- `ptn` - Path length
- `prn` - Profile length
- `s` - Generate start face at `path[0]`
- `e` - Generate end face at `path[ptn-1]`

### `tp3d_Sweep_EdgeFaces(ptn,prn,s=true,e=true)`

- Build closure faces by letting OpenSCAD sort out triangles for the edge faces
- `ptn` - Path length
- `prn` - Profile length
- `s` - Generate start face at `profile[0]`
- `e` - Generate end face at `profile[ptn-1]`

### `tp3d_Sweep_ZipEndFaces(ptn,prn,zip_start=undef,s=true,e=true)`

- Build closure face triangles by zipping together end points
- `ptn` - Path length
- `prn` - Profile length
- `zip_start` - Optional start index for zip operation
- `s` - Generate start face at `path[0]`
- `e` - Generate end face at `path[ptn-1]`

### `tp3d_Sweep_ZipEdgeFaces(ptn,prn,zip_start=undef,s=true,e=true)`

- Build closure face triangles by zipping together edge points
- `ptn` - Path length
- `prn` - Profile length
- `zip_start` - Optional start index for zip operation
- `s` - Generate start face at `profile[0]`
- `e` - Generate end face at `profile[ptn-1]`
