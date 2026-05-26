/*
	TurtlePath 3d
	-------------
	Path/profile generation via step by step instructions
	- Easier than managing and aligning rounded corner centers and angles manually
	- Named in homage to https://en.wikipedia.org/wiki/Turtle_graphics
	- Native 3d, supporting Yaw/Turn (left/right), Pitch (up/down) and Roll (cw/ccw)
	- The primary step parameters are relative to the current position and heading
*/
use <Assertions.scad>;
use <Rotate3dVector.scad>;
use <TurtlePath3d_exec.scad>;
use <Project3dVector.scad>;

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
// Extract point, hv and nv from the accumulated result
// - step: >= 0 to index from the start, < 0 to index 'backwards'

function tp3d_GetCursor(acc,debug_prefix="",step=-1) =
let(idx=step>=0 ? step : len(acc.x)+step)
assert(idx>=0 && idx<len(acc.x),debug_prefix)
[ acc.x[idx], acc.y[idx], acc.z[idx] ];

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
assert(is_num(d) && d>0,str(debug_prefix,":d=",d))
assert(is_num(n) && n>0,debug_prefix)
let(cu=tp3d_GetCursor(acc,debug_prefix))
[
	[ for(i=[1:n]) cu.x+cu.y*d*i/n ],
	[ for(i=[1:n]) cu.y ],
	[ for(i=[1:n]) cu.z ],
];

/***************************************************************************************/
// Step forward to intersection with a plane in n steps
// - pp: Point on plane
// - pnv: Plane normal vector
// - dd: Distance delta, added to distance to intersection with plane

function tp3d_ToPlane(acc,debug_prefix,pp,pnv,dd=0,n=1) =
assert(is_num(dd),debug_prefix)
let(c=tp3d_GetCursor(acc,debug_prefix),
	d=tp3d_DistToPlane(sp=c.x,hv=c.y,pp=pp,pnv=pnv,debug_prefix)+dd)
tp3d_Straight(acc,debug_prefix,d=d,n=n);

/***************************************************************************************/
// Calculate distance from sp along hv to intersection with plane with normal vector
// - sp: Starting point
// - hv: Heading vector
// - pp: Point on plane
// - pnv: Plane normal vector

function tp3d_DistToPlane(sp,hv,pp,pnv,debug_prefix="") =
let(check_hv=AssertValidVectorList([hv],3,require_nonzero_norm=true,prefix=debug_prefix),
	check_pnv=AssertValidVectorList([pnv],3,require_nonzero_norm=true,prefix=debug_prefix),
	hvu=hv/norm(hv),
	pnvu=pnv/norm(pnv),
	vPerpendicularToPlaneAndHeading=cross(hvu,pnvu),
	ppt=Project3dPointToPlane(pt=pp,pnv=vPerpendicularToPlaneAndHeading,ppt=sp,debug_prefix=debug_prefix),
	dSP=norm(VectorFrom3dPointToPlane(pt=sp,pnv=pnvu,ppt=ppt,debug_prefix=debug_prefix)),
	// a . b = |a| |b| cos(a)
	// cos(a) = adj/hyp  -->  hyp = adj/cos(a)
	)
abs(dSP/(hvu*pnvu));

/***************************************************************************************/
// Turn horizontally by angle a (positive:ccw,negative:cw) with radius r in n steps

function tp3d_Turn(acc,debug_prefix,a,r,dn=undef,n=10) =
assert(is_list(acc) && len(acc)>=3,debug_prefix)
assert(is_num(a) && a!=0,debug_prefix)
assert(is_num(r) && r>0,debug_prefix)
assert(is_num(n) && n>0,debug_prefix)
assert(is_undef(dn) || is_num(dn) || is_function(dn),debug_prefix)
let(cu=tp3d_GetCursor(acc,debug_prefix),
	cv=cross(cu.z,cu.y)*sign(a),
	cvr=r*cv/norm(cv),
	c=cu.x+cvr,
	q=Rotation3dAxis_Quaternion(cu.z,a/n,debug_prefix),
	turn_cvr=tp3d_rotateSteps(-cvr,q,n,debug_prefix))
[
	[
		for(i=[0:len(turn_cvr)-1])
		let(d=is_num(dn) ? dn*(i+1)/n
			: is_function(dn) ? dn((i+1)/n)
			: 0)
		assert(is_num(d),debug_prefix)
		c + turn_cvr[i] + cu.z*d
	],
	[ each tp3d_rotateSteps(cu.y,q,n,debug_prefix) ],
	[ for(i=[1:n]) cu.z ],
];

/***************************************************************************************/
// Pitch vertically by angle a (positive:up,negative:down) with radius r in n steps

function tp3d_Pitch(acc,debug_prefix,a,r,ds=undef,n=10) =
assert(is_list(acc) && len(acc)>=3,debug_prefix)
assert(is_num(a) && a!=0,debug_prefix)
assert(is_num(r) && r>0,debug_prefix)
assert(is_num(n) && n>0,debug_prefix)
assert(is_undef(ds) || is_num(ds) || is_function(ds),debug_prefix)
let(cu=tp3d_GetCursor(acc,debug_prefix),
	sv=-cross(cu.z,cu.y),
	cvr=r*cu.z*sign(a),
	c=cu.x+cvr,
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
	[ each tp3d_rotateSteps(cu.y,q,n,debug_prefix) ],
	[ each tp3d_rotateSteps(cu.z,q,n,debug_prefix) ],
];

/***************************************************************************************/
// Roll around hv by angle a (positive:ccw,negative:cw) in d mm in n steps

function tp3d_Roll(acc,debug_prefix,a,d,n=10) =
assert(is_list(acc) && len(acc)>=3,debug_prefix)
assert(is_num(a) && a!=0,debug_prefix)
assert(is_num(d) && d>0,debug_prefix)
assert(is_num(n) && n>0,debug_prefix)
let(cu=tp3d_GetCursor(acc,debug_prefix),
	q=Rotation3dAxis_Quaternion(cu.y,-a/n,debug_prefix))
[
	[ for(i=[1:n]) cu.x + cu.y*d*i/n ],
	[ for(i=[1:n]) cu.y ],
	[ each tp3d_rotateSteps(cu.z,q,n,debug_prefix) ],
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
let(cu=tp3d_GetCursor(acc,debug_prefix),
	rv=Rotate3dVector_hvr(cu.y,cu.z,a_horz,a_vert,a_roll,debug_prefix))
tp3d_PivotTo(acc,debug_prefix,rv.x,rv.y);
