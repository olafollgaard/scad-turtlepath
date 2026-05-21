/*
	TurtlePath 3d tests
*/
use <TurtlePath3d.scad>;
use <TurtlePath3d_Sweep.scad>;

test_profile=[for(a=[0:4]) let(r=1.2,a=270-72*a) r*[cos(a),sin(a)]];
test_innerpr=[for(a=[0:4]) let(r=1.0,a=270-72*a) r*[cos(a),sin(a)]];

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
tp3d_Sweep(profile=pr,path=pt);

translate([5,0,0])
let(pr=test_profile,
	pt=TurtlePath3d(steps=[
		function(acc,dp) tp3d_Straight(acc,dp,d=3,n=2),
		function(acc,dp) tp3d_Pivot(acc,dp,a_roll=-15),
	]))
tp3d_Sweep(profile=pr,path=pt);

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
tp3d_Sweep(profile=tp3d_GetPathXY(pr),path=pt,closure=["plate-edge"]);
