/*
	TurtlePath 3d internal
*/
use <Assertions.scad>;

// Execute steps and concatenate into full path
function tp3d_execPath(steps,step_n,st,hv,nv,debug_prefix,incl_debug_labels=false) =
let(acc=step_n==0 ? [[st],[hv],[nv],[],[]] : tp3d_execPath(steps,step_n-1,st,hv,nv,debug_prefix,incl_debug_labels),
	step_debug_prefix=str(debug_prefix,".",step_n),
	curr=tp3d_execStep(acc,steps[step_n],step_debug_prefix),
	path_pts=[ each acc.x, each curr.x ],
	path_hvs=len(curr.x)>0 || len(curr.y)==0 ? [ each acc.y, each curr.y ]
	: [ for(i=[0:len(acc.y)-2]) acc.y[i], curr.y[0] ], // pivot hv
	path_nvs=len(curr.x)>0 || len(curr.z)==0 ? [ each acc.z, each curr.z ]
	: [ for(i=[0:len(acc.z)-2]) acc.z[i], curr.z[0] ], // pivot nv
	labels=[ each acc[3], if(incl_debug_labels) [step_debug_prefix,len(acc.x)-1], each curr[3] ],
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

// Generate a 2d path from the accumulated/main result
function tp3d_execGetPathXY(acc,skip_last=false) =
[ for(i=[0:len(acc.x)-(skip_last ? 2 : 1)]) let(p=acc.x[i]) [p.x,p.y] ];

// Extract point, hv and nv from the accumulated result
function tp3d_execGetCursor(acc,debug_prefix="",step=-1) =
let(idx=step>=0 ? step : len(acc.x)+step)
assert(idx>=0 && idx<len(acc.x),debug_prefix)
[ acc.x[idx], acc.y[idx], acc.z[idx] ];

// Lookup the step index of a given label
function tp3d_execIdxByLabel(label,acc,debug_prefix="") =
assert(is_string(label),debug_prefix)
assert(is_list(acc) && len(acc)>3 && is_list(acc[3]),debug_prefix)
let(found=[ for(p=acc[3]) if(p.x==label) p.y ])
assert(len(found)>0,str(debug_prefix,":unknown label '",label,"'"))
found.x;
