/*
	Dovetail module and edge function
*/
use <TurtlePath3d.scad>;
use <TurtlePath3d_Sweep.scad>;

let(dtmode="test",//test|print
	clr=0.1,
	clr_wr=0.05,
	sz=[10,30,10],
	tab_ow=10,
	tab_dwi=2,
	tab_l=3,
	tab_cr=0.4,
	tab_dwz=2)
{
	translate(dtmode=="test"?[+clr/2,0,0]:[+tab_l,0,0])
	DovetailBlock(clr=+clr,clr_wr=clr_wr,sz=sz,tab_ow=tab_ow,tab_dwi=tab_dwi,tab_l=tab_l,tab_cr=tab_cr,tab_dwz=tab_dwz);
	translate(dtmode=="test"?[-clr/2,0,0]:[-tab_l,0,0])
	DovetailBlock(clr=-clr,clr_wr=clr_wr,sz=sz,tab_ow=tab_ow,tab_dwi=tab_dwi,tab_l=tab_l,tab_cr=tab_cr,tab_dwz=tab_dwz);
}

/***************************************************************************************/
// Dovetail block module
// - clr: Clearance - positive for external, negative for internal
// - clr_wr: Reduce tab width clearance by this value
// - sz: Block size - dovetail is on the YZ plane
// - tab_ow: Outer width of external tab
// - tab_dwi: Delta width, or how much smaller should the inner width be
// - tab_l: Length of external tab
// - tab_cr: Tab corner radius
// - tab_dwz: Delta width along Z

module DovetailBlock(clr=0.1,clr_wr=0.05,sz=[10,20,10],tab_ow=10,tab_dwi=2,tab_l=3,tab_cr=0.4,tab_dwz=0)
assert(clr!=0) // sign of clr determines external(+) / internal(-) dovetail
let(dey=sz.y/2,
	pt=TurtlePath3d(debug_prefix="Dovetail path",steps=[
		function(acc,dp) tp3d_Straight(acc,dp,d=sz.z)
	]),
	prf=function(step)
		let(dy=step==0 ? 0 : tab_dwz,
			dte=DovetailEdge(clr=clr,clr_wr=clr_wr,szy=sz.y,tab_ow=tab_ow+dy,tab_dwi=tab_dwi,tab_l=tab_l,tab_cr=tab_cr))
		clr > 0
		? [ [+sz.x,+dey], [+sz.x,-dey], each dte ]
		: [ [-sz.x,-dey], [-sz.x,+dey], each dte ],
	)
{
	rotate([90,0,0])
	TurtlePath3d_Sweep(debug_prefix="Dovetail",path=pt,profile=prf);
}

/***************************************************************************************/
// Dovetail edge profile function
// Returns list of 2d points, from bottom to top for external, from top to bottom for internal
// - clr: Clearance - positive for external, negative for internal
// - clr_wr: Reduce tab width clearance by this value
// - szy: Edge length
// - tab_ow: Outer width of external tab
// - tab_dwi: Delta width, or how much smaller should the inner width be
// - tab_l: Length of external tab
// - tab_cr: Tab corner radius

function DovetailEdge(clr=0.1,clr_wr=0.05,szy=20,tab_ow=10,tab_dwi=2,tab_l=3,tab_cr=0.4) =
assert(clr!=0) // sign of clr determines external(+) / internal(-) dovetail
assert(clr_wr>=0)
let(diy=tab_ow/2-tab_dwi/2+sign(clr)*clr_wr/2,
	dey=szy/2,
	cr_a=atan2(tab_dwi/2+abs(clr/2),tab_l-2*tab_cr),
	pr_sp=[clr/2,-dey,0],
	dt_p=TurtlePath3d(debug_prefix="Dovetail profile",start_pt=pr_sp,steps=[
		function(acc,dp) tp3d_AddLabel(acc,dp,"st"),
		function(acc,dp) tp3d_Straight(acc,dp,d=dey-diy-tab_cr),
		function(acc,dp) tp3d_Turn(acc,dp,a=90+cr_a,r=tab_cr+clr/2),
		function(acc,dp) tp3d_ToPlane(acc,dp,pp=[-tab_l+tab_cr,0,0],pnv=[-1,0,0],dd=-tan(cr_a)*(tab_cr-clr/2)),
		function(acc,dp) tp3d_Turn(acc,dp,a=-cr_a-90,r=tab_cr-clr/2),
		function(acc,dp) tp3d_Mirror(acc,dp,stop="st",mpt=[0,0,0]),
	]),
	dt2d=tp3d_GetPathXY(dt_p),
	n=len(dt2d))
clr > 0
? [ for(p=[1:n]) dt2d[p-1]-[clr/2,0] ]
: [ for(p=[1:n]) dt2d[n-p]-[clr/2,0] ]
;
