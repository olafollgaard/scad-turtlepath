/*
	TurtlePath 3D
*/
use <Assertions.scad>;
use <Rotate3dVector.scad>;

function TurtlePath3d(
	steps,
	// Start 3d point
	start_pt=[0,0,0],
	// Initial heading vector ('forward') in the 'horizontal' plane
	start_hv=[0,1,0],
	// Initial normal vector ('up') for the 'horizontal' plane
	start_nv=[0,0,1],
	// Prefix for assertion messages
	debug_name = "Path3d") =
assert(is_list(steps) && len(steps)>0,debug_name)
assert(len(AssertValidVectorList([start_pt],3,prefix=debug_name))==1,debug_name)
assert(len(AssertValidVectorList([start_hv],3,prefix=debug_name))==1,debug_name)
assert(len(AssertValidVectorList([start_nv],3,prefix=debug_name))==1,debug_name)
let(s_hv=start_hv/norm(start_hv),
	s_nv=start_nv/norm(start_nv))
assert((s_hv*s_nv)!=1,str(debug_name,": start_hv and start_nv must not be parallel"))
let(nv_proj=start_hv*(start_hv*start_nv)/(start_hv*start_hv),
	nv_perp=start_nv-nv_proj)
tp3d_execPath(steps,
	step_n=len(steps)-1,
	st=start_pt,
	hv=start_hv/norm(start_hv),
	nv=nv_perp/norm(nv_perp),
	debug_name=debug_name);

// Execute steps and concatenate into full path
function tp3d_execPath(steps,step_n,st,hv,nv,debug_name) =
let(acc=step_n==0 ? [[st],[hv],[nv],[],[]] : tp3d_execPath(steps,step_n-1,st,hv,nv,debug_name),
	step_debug_prefix=str(debug_name,".",step_n),
	curr=tp3d_execStep(acc,steps[step_n],step_debug_prefix),
	path_pts=[ each acc.x, each curr.x ],
	path_hvs=len(curr.x)>0 || len(curr.y)==0 ? [ each acc.y, each curr.y ]
	: [ for(i=[0:len(acc.y)-2]) acc.y[i], curr.y[0] ],
	path_nvs=len(curr.x)>0 || len(curr.z)==0 ? [ each acc.z, each curr.z ]
	: [ for(i=[0:len(acc.z)-2]) acc.z[i], curr.z[0] ],
	labels=[ each acc[3], [step_debug_prefix,len(acc.x)-1], each curr[3] ],
	stored=[ each acc[4], each curr[4] ])
[ path_pts, path_hvs, path_nvs, labels, stored ];

// Execute single step
function tp3d_execStep(acc,step,debug_prefix) =
assert(is_function(step),debug_prefix)
let(res=step(acc,debug_prefix))
assert(is_list(res) && len(res)>=1,debug_prefix)
let(step_pts=is_list(res.x) && len(res.x)==0 ? []
	: AssertValidVectorList(res.x,3,prefix=debug_prefix),
	step_hvs=len(res)>=2 && is_list(res.y) && len(res.y)>0
	? AssertValidVectorList(res.y,3,require_nonzero_norm=true,prefix=debug_prefix)
	: [ for(i=[1:len(step_pts)]) acc.y[len(acc.y)-1] ],
	step_nvs=len(res)>=3 && is_list(res.z) && len(res.z)>0
	? AssertValidVectorList(res.z,3,require_nonzero_norm=true,prefix=debug_prefix)
	: [ for(i=[1:len(step_pts)]) acc.z[len(acc.z)-1] ])
assert(len(step_hvs)==len(step_nvs),debug_prefix)
assert(len(step_pts)==0 ? len(step_hvs)<=1 : len(step_hvs)==len(step_pts),debug_prefix)
let(perp_nvs=[
		for(i=[0:len(step_nvs)-1])
		let(s_hv=step_hvs[i]/norm(step_hvs[i]),
			s_nv=step_nvs[i]/norm(step_nvs[i]))
		assert((s_hv*s_nv)!=1,str(debug_prefix,"@",i,": hv and nv must not be parallel"))
		s_nv-s_hv*(s_hv*s_nv)/(s_hv*s_hv)
	],
	step_labels=len(res)<=3 ? []
	: assert(is_list(res[3]),debug_prefix) [
		for(i=res[3])
		assert(is_list(i) && len(i)==2 && is_string(i.x) && is_num(i.y),debug_prefix)
		assert(len([for(p=acc[3]) if(p.x==i.x) p])==0,str(debug_prefix,": duplicate label '",i.x,"'"))
		i
	],
	step_stored=len(res)<=4 ? []
	: assert(is_list(res[4]),debug_prefix) [
		for(i=res[4])
		assert(is_list(i) && len(i)==2 && is_string(i.x),debug_prefix)
		i
	])
[
	step_pts,
	[ for(hv=step_hvs) hv/norm(hv) ],
	[ for(nv=perp_nvs) nv/norm(nv) ],
	step_labels,
	step_stored,
];

// Add label for previous step
function tp3d_AddLabel(acc,debug_prefix,name,di=0) =
assert(is_string(name),debug_prefix)
assert(is_num(di),debug_prefix)
[
	[],[],[],[[name,len(acc.x)-1+di]]
];

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
