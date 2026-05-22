/*
	TurtlePath 3d Sweep
*/
use <Assertions.scad>;

/***************************************************************************************/
// Sweep module, building a solid from TurtlePath3d results

module TurtlePath3d_Sweep(
	// profile points, either as a simple list of 2d points or as a lambda
	// expression <function(i)> that must generate a list of 2d points
	// - all results of lambda must have same length
	profile,
	// path as returned by TurtlePath3d()
	path,
	// closure modes:
	// - [ "bar" ] or [ "face-ends" ]
	//		- use points generated from path.x[0] as edge of start face
	//		- use points generated from path.x[n-1] as edge of end face
	//		- add all face points together, for OpenSCAD to sort out triangles
	// - [ "zip-ends", <startidx=0> ]
	//		loop the profile points
	//		- use points generated from path.x[0] as edge of start face
	//		- use points generated from path.x[n-1] as edge of end face
	//		- zip profile points together, starting from 0 or given profile index
	// - [ "torus" ] or [ "path-loop" ]
	//		loop both profile points and path points
	//		- there are no end faces, result will be a torus
	// - [ "plate-edge" ] or [ "face-edges" ]
	//		loop the path points, but not the profile points
	//		- use generated points from profile[0] along path as edge of 'start' face
	//		- use generated points from profile[n-1] along path as edge of 'end' face
	//		- add all face points together, for OpenSCAD to sort out triangles
	// - [ "zip-edges" ]
	//		loop the path points, but not the profile points
	//		- use generated points from profile[0] along path as edge of 'start' face
	//		- use generated points from profile[n-1] along path as edge of 'end' face
	// - [ "tube", <inner_profile> ]
	//		use the given inner profile to create the inside wall of the tube
	//		- the inner profile must be equivalent to the outer profile, i.e. it must
	//			be either a list of 2d points or a <function(i)> to generate one
	//		- the inner profile must have the same number of points as the outer one
	//		- profiles are looped and must be path-loop compatible
	// - [ "cap-tube", <inner_profile>, <cap-step-count=1> ]
	//		use the given inner profile to create the inside wall of the tube
	//		- the inner profile must be equivalent to the outer profile, i.e. it must
	//			be either a list of 2d points or a <function(i)> to generate one
	//		- the inner profile must have the same number of points as the outer one
	//		- profiles are looped
	//		- use outer profile from path.x[0] as edge of the outside cap face
	//		- use inner profile from path.x[<cap-step-count>] as edge of the
	//			inside cap face
	closure=["bar"],
	// prefix for assertions
	debug_prefix="sweep")
assert(is_list(profile) || is_function(profile),debug_prefix)
assert(is_list(path) && len(path)>=3,debug_prefix)
let(check_pts=AssertValidVectorList(path.x,3,prefix=debug_prefix),
	check_hv=AssertValidVectorList(path.y,3,require_nonzero_norm=true,prefix=debug_prefix),
	check_nv=AssertValidVectorList(path.z,3,require_nonzero_norm=true,prefix=debug_prefix))
assert(is_list(closure) && len(closure)>0 && is_string(closure[0]),debug_prefix)
let(closure_type=closure[0])
assert(closure_type=="bar" || closure_type=="face-ends" || closure_type=="zip-ends"
	|| closure_type=="torus" || closure_type=="path-loop"
	|| closure_type=="plate-edge" || closure_type=="face-edges" || closure_type=="zip-edges"
	|| closure_type=="tube" || closure_type=="cap-tube",
	debug_prefix)
let(outer_profile=profile,
	outer_profile_pts=is_list(outer_profile) ? outer_profile : outer_profile(0),
	prn=len(AssertValidVectorList(outer_profile_pts,2,prefix=debug_prefix)),
	ptn=len(path.x),
	inner_profile=closure_type!="tube" && closure_type!="cap-tube" ? undef : closure.y,
	inner_profile_pts=closure_type!="tube" && closure_type!="cap-tube" ? undef
	: AssertValidVectorList(is_list(inner_profile) ? inner_profile : inner_profile(0),2,prefix=debug_prefix))
assert(closure_type=="tube" || closure_type=="cap-tube"
	? len(inner_profile_pts)==len(outer_profile_pts) : true,debug_prefix)
let(path_loop= closure_type=="torus" || closure_type=="path-loop"
		|| closure_type=="plate-edge" || closure_type=="face-edges" || closure_type=="zip-edges"
		|| closure_type=="tube",
	profile_loop= closure_type!="plate-edge" && closure_type!="face-edges" && closure_type!="zip-edges",
	pidx=function(pri,pti) pri%prn + (pti%ptn)*prn,
	outer_pts=tp3d_SweepProfilePoints(path=path,debug_prefix=debug_prefix,profile=outer_profile),
	inner_pts_raw=closure_type!="tube" && closure_type!="cap-tube" ? []
	: tp3d_SweepProfilePoints(path=path,debug_prefix=debug_prefix,profile=inner_profile),
	inner_cap_pt=closure_type!="cap-tube" ? 0
	: !is_num(closure.z) ? 1 : assert(closure.z>0 && closure.z<ptn,debug_prefix) closure.z,
	inner_pts=closure_type!="tube" && closure_type!="cap-tube" ? []
	: [
		for(pt=[ptn-1:-1:inner_cap_pt])
		for(pr=[0:prn-1])
		inner_pts_raw[pidx(pr,pt)]
	],
	wallfaces_ptn=closure_type=="tube" ? 2*ptn
	: closure_type=="cap-tube" ? 2*ptn-inner_cap_pt
	: ptn,
	points=
	[
		each outer_pts, each inner_pts,
	],
	even=prn%2==0,
	zs=closure_type=="zip-ends" && is_num(closure.y) ? closure.y : 0,
	zn=closure_type=="zip-ends" ? floor(prn/2)
	: closure_type=="zip-edges" ? floor(ptn/2) : 0,
	faces=[
		each tp3d_Sweep_WallFaces(profile_len=prn,path_len=wallfaces_ptn,path_loop=path_loop,profile_loop=profile_loop,debug_prefix=debug_prefix),
		each closure_type=="zip-ends" ?
		let(zLb=[for(i=[0:zn-1]) pidx(zs+i,0)],
			zRb=[for(i=[0:zn-1]) pidx(zs+prn-1-i,0)],
			zLe=[for(i=[0:zn-1]) pidx(zs+i,ptn-1)],
			zRe=[for(i=[0:zn-1]) pidx(zs+prn-1-i,ptn-1)])
		[
			for(i=[0:zn-2])
			each [
				[ zLb[i], zLb[i+1], zRb[i] ],
				[ zLb[i+1], zRb[i+1], zRb[i] ],
				[ zLe[i], zRe[i], zLe[i+1] ],
				[ zLe[i+1], zRe[i], zRe[i+1] ]
			],
			if (prn%2>0)
			each [
				[ zRb[zn-1], zLb[zn-1], pidx(zs+zn,0) ],
				[ zLe[zn-1], zRe[zn-1], pidx(zs+zn,ptn-1) ],
			]
		]
		: closure_type=="bar" || closure_type=="face-ends" ? [
			[for(i=[0:prn-1]) pidx(i,0)],
			[for(i=[prn-1:-1:0]) pidx(i,ptn-1)]
		]
		: closure_type=="zip-edges" ?
		let(zLb=[for(i=[0:zn-1]) pidx(0,zs+i)],
			zRb=[for(i=[0:zn-1]) pidx(0,zs+ptn-1-i)],
			zLe=[for(i=[0:zn-1]) pidx(prn-1,zs+i)],
			zRe=[for(i=[0:zn-1]) pidx(prn-1,zs+ptn-1-i)])
		[
			for(i=[0:zn-2])
			each [
				[ zLb[i+1], zLb[i], zRb[i] ],
				[ zRb[i+1], zLb[i+1], zRb[i] ],
				[ zRe[i], zLe[i], zLe[i+1] ],
				[ zRe[i], zLe[i+1], zRe[i+1] ]
			],
			if (ptn%2>0)
			each [
				[ zLb[zn-1], zRb[zn-1], pidx(0,zs+zn) ],
				[ zRe[zn-1], zLe[zn-1], pidx(prn-1,zs+zn) ],
			]
		]
		: closure_type=="plate-edge" || closure_type=="face-edges" ? [
			[for(i=[ptn-1:-1:0]) pidx(0,i)],
			[for(i=[0:ptn-1]) pidx(prn-1,i)],
		]
		: closure_type=="cap-tube" ?
		let(cpidx=function(pri,pti) pri%prn + (pti%wallfaces_ptn)*prn)
		[
			[for(i=[0:prn-1]) cpidx(i,0)],
			[for(i=[prn-1:-1:0]) cpidx(i,wallfaces_ptn-1)]
		]
		: //closure_type=="torus" || closure_type=="path_loop" || closure_type=="tube"
		[]
	])
{
	polyhedron(points=points, faces=faces, convexity=10);
}

/*
	Build wall points for polyhedron
	- Profile points must be ordered clockwise
	- Warns about hard turns on the path
*/
function tp3d_SweepProfilePoints(
	path,
	loop=false,
	debug_prefix="Sweep3dProfile",
	profile=function(step) []) =
let(first_profile_pts=is_list(profile)
	? profile
	: AssertValidVectorList(profile(0),2,prefix=debug_prefix),
	prn=len(first_profile_pts),
	steps=[
		for(step=[0:len(path.x)-1])
		let(spr=is_list(profile) ? profile : AssertValidVectorList(profile(step),2,prefix=debug_prefix))
		assert(len(spr)==prn,str(debug_prefix,"@",step,":expected profile length ",prn,", found ",len(spr)))
		let(cp=path.x[step],
			hv=path.y[step],
			nv=path.z[step],
			sv=-cross(nv,hv),
			pts=[ for(p=spr) cp + p.x*sv + p.y*nv ],
			check_pts=step==0 ? [] : [
			 	for(i=[0:prn-1])
			 	let(dpc=pts[i]-path.x[step-1],
			 		dpcu=dpc/norm(dpc),
					phv=path.y[step-1])
			 	if((phv*dpcu)<=0)
			 	echo(str(debug_prefix,"@",step,".",i,":path curve may be too sharp for the profile"))
			 	true
			],
			)
		pts
	])
[ for (step_pts=steps) each step_pts ];

/*
	Build wall faces for polyhedron
	- only profile walls along the path, does not cap the ends
*/
function tp3d_Sweep_WallFaces(
	// profile length
	profile_len,
	// path length
	path_len,
	// closed loop path
	path_loop=false,
	// closed loop profile
	profile_loop=true,
	// prefix for assertions
	debug_prefix="Sweep3dFaces") =
assert(is_num(profile_len),debug_prefix)
assert(is_num(path_len),debug_prefix)
assert(is_bool(path_loop),debug_prefix)
assert(is_bool(profile_loop),debug_prefix)
let(prn=profile_len,
	ptn=path_len,
	pidx=function(pri,pti) pri%prn + (pti%ptn)*prn)
[
	// populate walls
	for (p=[0:ptn-(path_loop?1:2)])
	for (i=[0:prn-(profile_loop?1:2)])
	each [
		[
			pidx(i+1,p+0),
			pidx(i+0,p+0),
			pidx(i+0,p+1),
		],
		[
			pidx(i+1,p+1),
			pidx(i+1,p+0),
			pidx(i+0,p+1),
		],
	],
];
