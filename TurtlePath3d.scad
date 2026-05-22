/*
	TurtlePath 3d
	-------------
	Path/profile generation via step by step instructions
	- Easier than managing and aligning rounded corner centers and angles manually
	- Named in homage to https://en.wikipedia.org/wiki/Turtle_graphics
	- This version is native 3d, supporting Yaw ("turn"), Pitch and Roll operations
	- Also, the primary step parameters are relative to the current heading, just
		like the original TurtleGraphics
*/
use <Assertions.scad>;
use <Rotate3dVector.scad>;
use <TurtlePath3d_exec.scad>;

/***************************************************************************************/
// Main TurtlePath3d function
// Returns: [ path_pts, path_hvs, path_nvs, labels, stored ]
// - path_pts: a 3d point for each step
// - path_hvs: a 3d heading vector for each step
// - path_nvs: a 3d 'horizontal' plane normal vector ('up') for each step
// - labels: a list of ["label",idx] pairs for step index lookup by name
// - stored: a list of ["name",value] pairs for custom storage

function TurtlePath3d(
	// steps: Step by step instructions, each of which must be a function(acc,dp):
	// - acc: The accumulated result so far, on the same format as the main result
	// - dp: Debugging prefix for assertion messages
	// Each step function must return its contribution to the main result, with only
	// the path_pts part being mandatory; the hv/nv vectors lists, if omitted, will be
	// automatically padded with the last hv/nv values.
	steps,
	// start_pt: Start 3d point
	start_pt=[0,0,0],
	// start_hv: Initial heading vector ('forward') in the 'horizontal' plane
	start_hv=[0,1,0],
	// start_nv: Initial normal vector ('up') for the 'horizontal' plane
	start_nv=[0,0,1],
	// drop_last: Drop last point in path, to avoid duplicate point when creating a loop
	drop_last=false,
	// debug_prefix: Prefix for assertion messages
	debug_prefix = "Path3d") =
assert(is_list(steps) && len(steps)>0,debug_prefix)
assert(len(AssertValidVectorList([start_pt],3,prefix=debug_prefix))==1,debug_prefix)
assert(len(AssertValidVectorList([start_hv],3,require_nonzero_norm=true,prefix=debug_prefix))==1,debug_prefix)
assert(len(AssertValidVectorList([start_nv],3,require_nonzero_norm=true,prefix=debug_prefix))==1,debug_prefix)
let(s_hv=start_hv/norm(start_hv),
	s_nv=start_nv/norm(start_nv))
assert((s_hv*s_nv)!=1,str(debug_prefix,": start_hv and start_nv must not be parallel"))
let(nv_proj=start_hv*(start_hv*start_nv)/(start_hv*start_hv),
	nv_perp=start_nv-nv_proj,
	pt=tp3d_execPath(steps,
		step_n=len(steps)-1,
		st=start_pt,
		hv=start_hv/norm(start_hv),
		nv=nv_perp/norm(nv_perp),
		debug_prefix=debug_prefix))
[
	[ for(i=[0:len(pt.x)-(drop_last ? 2 : 1)]) pt.x[i] ],
	for(i=[1:len(pt)-1]) pt[i],
];

/***************************************************************************************/
// Generate a 2d path from the accumulated/main result
// - Simply copies the X and Y parts of path_pts and drops the Z part
// - Intended for generating a profile for TurtlePath3d_Sweep() using TurtlePath3d()

function tp3d_GetPathXY(acc,skip_last=false) =
[ for(i=[0:len(acc.x)-(skip_last ? 2 : 1)]) let(p=acc.x[i]) [p.x,p.y] ];

/***************************************************************************************/
// Extract last point, hv and nv from the accumulated result

function tp3d_GetCursor(acc) =
let(sp=acc.x[len(acc.x)-1],
	hv=acc.y[len(acc.y)-1],
	nv=acc.z[len(acc.z)-1])
[ sp,hv,nv ];

/***************************************************************************************/
// Lookup the step index of a given label

function tp3d_IdxByLabel(label,acc,debug_prefix="") =
assert(is_string(label),debug_prefix)
assert(is_list(acc) && len(acc)>3 && is_list(acc[3]),debug_prefix)
let(found=[ for(p=acc[3]) if(p.x==label) p.y ])
assert(len(found)>0,str(debug_prefix,":unknown label '",label,"'"))
found.x;

/***************************************************************************************/
// Add label for previous step, with an optional 'delta' to adjust the recorded index

function tp3d_AddLabel(acc,debug_prefix,name,di=0) =
assert(is_string(name),debug_prefix)
assert(is_num(di),debug_prefix)
[
	[],[],[],[[name,len(acc.x)-1+di]]
];

/***************************************************************************************/
// Step forward d mm in n steps

function tp3d_Straight(acc,debug_prefix,d,n=1) =
assert(is_num(d) && d>0,debug_prefix)
assert(is_num(n) && n>0,debug_prefix)
let(sp=acc.x[len(acc.x)-1],
	hv=acc.y[len(acc.y)-1],
	nv=acc.z[len(acc.z)-1])
[
	[ for(i=[1:n]) sp+hv*d*i/n ],
	[ for(i=[1:n]) hv ],
	[ for(i=[1:n]) nv ],
];

/***************************************************************************************/
// Turn horizontally by angle a (positive:ccw,negative:cw) with radius r in n steps

function tp3d_Turn(acc,debug_prefix,a,r,dn=undef,n=10) =
assert(is_list(acc) && len(acc)>=3,debug_prefix)
assert(is_num(a) && a!=0,debug_prefix)
assert(is_num(r) && r>0,debug_prefix)
assert(is_num(n) && n>0,debug_prefix)
assert(is_undef(dn) || is_num(dn) || is_function(dn),debug_prefix)
let(sp=acc.x[len(acc.x)-1],
	hv=acc.y[len(acc.y)-1],
	nv=acc.z[len(acc.z)-1],
	cv=cross(nv,hv)*sign(a),
	cvr=r*cv/norm(cv),
	c=sp+cvr,
	q=Rotation3dAxis_Quaternion(nv,a/n,debug_prefix),
	turn_cvr=tp3d_rotateSteps(-cvr,q,n,debug_prefix))
[
	[
		for(i=[0:len(turn_cvr)-1])
		let(d=is_num(dn) ? dn*(i+1)/n
			: is_function(dn) ? dn((i+1)/n)
			: 0)
		assert(is_num(d),debug_prefix)
		c + turn_cvr[i] + nv*d
	],
	[ each tp3d_rotateSteps(hv,q,n,debug_prefix) ],
	[ for(i=[1:n]) nv ],
];

/***************************************************************************************/
// Pitch vertically by angle a (positive:up,negative:down) with radius r in n steps

function tp3d_Pitch(acc,debug_prefix,a,r,ds=undef,n=10) =
assert(is_list(acc) && len(acc)>=3,debug_prefix)
assert(is_num(a) && a!=0,debug_prefix)
assert(is_num(r) && r>0,debug_prefix)
assert(is_num(n) && n>0,debug_prefix)
assert(is_undef(ds) || is_num(ds) || is_function(ds),debug_prefix)
let(sp=acc.x[len(acc.x)-1],
	hv=acc.y[len(acc.y)-1],
	nv=acc.z[len(acc.z)-1],
	sv=-cross(nv,hv),
	cvr=r*nv*sign(a),
	c=sp+cvr,
	q=Rotation3dAxis_Quaternion(sv,a/n,debug_prefix),
	turn_cvr=tp3d_rotateSteps(-cvr,q,n,debug_prefix))
[
	[
		for(i=[0:len(turn_cvr)-1])
		let(d=is_num(ds) ? ds*(i+1)/n
			: is_function(ds) ? ds((i+1)/n)
			: 0)
		assert(is_num(d),debug_prefix)
		c + turn_cvr[i] + sv*d
	],
	[ each tp3d_rotateSteps(hv,q,n,debug_prefix) ],
	[ each tp3d_rotateSteps(nv,q,n,debug_prefix) ],
];

/***************************************************************************************/
// Roll around hv by angle a (positive:ccw,negative:cw) in d mm in n steps

function tp3d_Roll(acc,debug_prefix,a,d,n=10) =
assert(is_list(acc) && len(acc)>=3,debug_prefix)
assert(is_num(a) && a!=0,debug_prefix)
assert(is_num(d) && d>0,debug_prefix)
assert(is_num(n) && n>0,debug_prefix)
let(sp=acc.x[len(acc.x)-1],
	hv=acc.y[len(acc.y)-1],
	nv=acc.z[len(acc.z)-1],
	q=Rotation3dAxis_Quaternion(hv,-a/n,debug_prefix))
[
	[ for(i=[1:n]) sp + hv*d*i/n ],
	[ for(i=[1:n]) hv ],
	[ each tp3d_rotateSteps(nv,q,n,debug_prefix) ],
];

function tp3d_rotateSteps(v,q,n,debug_prefix) =
let(prev=n==1 ? [] : tp3d_rotateSteps(v,q,n-1,debug_prefix),
	vcurr=n==1 ? v : prev[len(prev)-1])
[ each prev, Rotate3dVector_UseQuaternion(vcurr,q,debug_prefix) ];

/***************************************************************************************/
// Pivot in place to given absolute hv and nv

function tp3d_PivotTo(acc,debug_prefix,hv,nv) =
assert(len(AssertValidVectorList([hv],3,require_nonzero_norm=true,prefix=debug_prefix))==1,debug_prefix)
assert(len(AssertValidVectorList([nv],3,require_nonzero_norm=true,prefix=debug_prefix))==1,debug_prefix)
[
	[], [hv/norm(hv)], [nv/norm(nv)]
];

/***************************************************************************************/
// Pivot in place by given angles relative to the 'horizontal' plane
// - a_horz: angle to pivot 'left'/'right'
// - a_vert: angle to pivot 'up'/'down'
// - a_roll: angle to roll 'ccw'/'cw'
// Pivot rotations are performed in the order 'horz', 'vert', 'roll'

function tp3d_Pivot(acc,debug_prefix,a_horz=0,a_vert=0,a_roll=0) =
assert(is_list(acc) && len(acc)>=3,debug_prefix)
assert(is_num(a_horz) && is_num(a_vert) && is_num(a_roll),debug_prefix)
let(hv_org=acc.y[len(acc.y)-1],
	nv_org=acc.z[len(acc.z)-1],
	hv_h=a_horz==0 ? hv_org : Rotate3dVector(v=hv_org,k=nv_org,a=a_horz,debug_prefix=debug_prefix),
	nv_h=nv_org,
	sv_h=-cross(nv_h,hv_h),
	hv_v=a_vert==0 ? hv_h : Rotate3dVector(v=hv_h,k=sv_h,a=a_vert,debug_prefix=debug_prefix),
	nv_v=a_vert==0 ? nv_h : Rotate3dVector(v=nv_h,k=sv_h,a=a_vert,debug_prefix=debug_prefix),
	hv_r=hv_v,
	nv_r=a_roll==0 ? nv_v : Rotate3dVector(v=nv_v,k=hv_v,a=-a_roll,debug_prefix=debug_prefix),
	)
tp3d_PivotTo(acc,debug_prefix,hv_r,nv_r);
