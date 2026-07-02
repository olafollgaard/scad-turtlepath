/*
	TurtlePath 3d tests
*/
use <TurtlePath3d.scad>;
use <TurtlePath3d_Sweep.scad>;

test_profile=[for(a=[0:4]) let(r=1.2,a=270-72*a) r*[cos(a),sin(a)]];
test_innerpr=[for(a=[0:4]) let(r=1.0,a=270-72*a) r*[cos(a),sin(a)]];

/*/ multistep test
let(pr=test_profile,
	pt=TurtlePath3d(steps=[
		function(acc,dp) tp3d_Straight(acc,dp,d=3,n=2),
		function(acc,dp) tp3d_AddLabel(acc,dp,name="test",di=42),
		function(acc,dp) tp3d_Turn(acc,dp,a=-45,r=2),
		function(acc,dp) tp3d_Turn(acc,dp,a=470,r=2,dn=5,n=20),
		function(acc,dp) tp3d_Pitch(acc,dp,a=+90,r=2),
		function(acc,dp) tp3d_Roll(acc,dp,a=-30,d=2),
	]))
echo(str("labels: ",pt[3]))
TurtlePath3d_Sweep(profile=pr,path=pt);
/**/

/*/ pivot twist
translate([5,0,0])
let(pr=test_profile,
	pt=TurtlePath3d(steps=[
		function(acc,dp) tp3d_Straight(acc,dp,d=3,n=2),
		function(acc,dp) tp3d_Pivot(acc,dp,a_roll=-15),
	]))
TurtlePath3d_Sweep(profile=pr,path=pt);
/**/

/*/ skewed plate edge
let(pr=TurtlePath3d(start_hv=[1,1,0],steps=[
		function(acc,dp) tp3d_Straight(acc,dp,d=2),
		function(acc,dp) tp3d_Pivot(acc,dp,a_horz=45),
		function(acc,dp) tp3d_Turn(acc,dp,a=-225,r=2,n=20),
		function(acc,dp) tp3d_Straight(acc,dp,d=2),
	]),
	pt=TurtlePath3d(start_pt=[10,0,-4],steps=[
		function(acc,dp) tp3d_Turn(acc,dp,a=270,r=10,n=20,dn=4),
		function(acc,dp) tp3d_Turn(acc,dp,a=-90,r=6,n=10),
		function(acc,dp) tp3d_Turn(acc,dp,a=180,r=7,n=20,dn=-4),
		function(acc,dp) tp3d_Turn(acc,dp,a=90,r=2,n=10),
		function(acc,dp) tp3d_Turn(acc,dp,a=-90,r=8,n=10),
	]))
TurtlePath3d_Sweep(profile=tp3d_GetPathXY(pr),path=pt,closure=["plate-edge"]);
/**/

/*/ ribbon faces
let(pr=-test_profile,
	pt=TurtlePath3d(steps=[
		function(acc,dp) tp3d_Turn(acc,dp,a=-335,r=5,n=15)
	]),
	prn=len(pr),
	ptn=len(pt.x),
	pts=tp3d_SweepProfilePoints(path=pt,profile=pr),
	wfcs=tp3d_Sweep_WallFaces(profile_len=prn,path_len=ptn,profile_loop=false,path_loop=true),
	edges=tp3d_Sweep_GetEdges(profile_len=prn,path_len=ptn,start_idx=0,list_len=4),
	rfcs=tp3d_Sweep_RibbonFaces(L=edges.y.x,R=edges.y.y))
{
	%polyhedron(points=pts,faces=wfcs,convexity=10);
	polyhedron(points=pts,faces=rfcs,convexity=10);
}
/**/

// bezier curve
let(pr=test_profile,
	pt=TurtlePath3d(steps=[
		function(acc,dp) tp3d_Bezier(acc,dp,sd=5,ept=[3,5,3],ehv=[1,0,0],env=[0,sin(36),cos(36)],ed=6,n=20),
	]))
TurtlePath3d_Sweep(profile=pr,path=pt);
/**/
