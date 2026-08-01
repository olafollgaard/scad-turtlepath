# Step functions

The first two parameters of all step functions are `acc` and `debug_prefix`, and they should always be given the two parameters of the `function(acc,dp)` step instruction unmodified.

### `tp3d_AddLabel(acc,dp,name,di=0)`

- Add label for a step index
- `name` - Label for previous step - duplicates not allowed
- `di` - Optional 'delta' to adjust the recorded index backward (negative) or forward (positive)

### `tp3d_AddStored(acc,dp,name,value)`

- Store `value` with the given `name` - duplicates not allowed

### `tp3d_Straight(acc,dp,d,n=1)`

- Step forward `d` mm in `n` steps

### `tp3d_ToPlane(acc,dp,pp,pnv,dd=0,n=1)`

- Step forward to intersection with a plane in `n` steps
- `pp` - Point on plane
- `pnv` - Plane normal vector
- `dd` - Distance delta, added to distance to intersection with plane

### `tp3d_Turn(acc,dp,a,r,dn=undef,n=10)`

- Turn 'horizontally' by angle `a` (positive:ccw,negative:cw) with radius `r` in `n` steps
- `dn` - Translate steps along normal vector
	- A simple number - Translate `dn` mm during turn, `dn/n` mm per step
	- A `function(pct)` - Call function for each step to get its translation in mm
		- `pct` - turn completion percentage
		- return translation in mm compared to current position, not individual step translation

### `tp3d_Pitch(acc,dp,a,r,ds=undef,n=10)`

- Pitch 'up/down' by angle `a` (positive:up,negative:down) with radius `r` in `n` steps
- `ds` - Same as `dn` in `tp3d_Turn`, but with sideways translation instead of vertical

### `tp3d_Roll(acc,dp,a,d,n=10)`

- Roll around current heading vector by angle `a` (positive:ccw,negative:cw) in `d` mm in `n` steps

### `tp3d_PivotTo(acc,dp,hv,nv)`

- Pivot in place to given absolute `hv` and `nv`

### `tp3d_Pivot(acc,dp,a_horz=0,a_vert=0,a_roll=0)`

- Pivot in place by given angles relative to current heading and normal vector
- `a_horz` - Angle to pivot 'left'/'right'
- `a_vert` - Angle to pivot 'up'/'down'
- `a_roll` - Angle to roll 'ccw'/'cw'
- Pivot rotations are performed in the order horz, vert, roll

### `tp3d_Bezier(acc,dp,sd,ept,ehv,env,ed,n=10,cosfade=true)`

- Step along a cubic Bezier curve in n steps ([Wiki](https://en.wikipedia.org/wiki/B%C3%A9zier_curve))
- `sd`: Distance along current heading to the first intermediate point
- `ept`: End point
- `ehv`: Heading vector from the end point forward
- `env`: Normal vector from the end point up
- `ed`: Distance from ept backwards along ehv to the second intermediate point
- `cosfade`: Use cosfade instead of linear interpolation in last step to improve alignment with current and end heading

### `tp3d_Mirror(acc,dp,stop,mpt=undef,mnv=undef)`

- Mirror preceding points back to given `stop` label
- `stop` - Label (added with `tp3d_AddLabel()`) at which to stop the mirroring
- `mpt` - Point on the mirror plane, defaults to current position
- `mnv` -  Mirror plane normal vector, defaults to current heading vector
