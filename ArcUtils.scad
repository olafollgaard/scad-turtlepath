/*
	Arc utils

	   .    <
	  /      |
	 (       | arc_w
	  \      |
	   '    <

	 ^-^
	arc_h
*/

/*****************************************************************************/
function ArcRadius( arc_h, arc_w, debug_prefix="" ) =
assert(arc_h<arc_w/2,str(debug_prefix,":arc_h (",arc_h,") must be less than half of arc_w (",arc_w,")"))
/*
	(R-H)^2 + (W/2)^2 = R^2
	R^2 + H^2 - 2*R*H + (W/2)^2 = R^2
		H^2 - 2*R*H + (W/2)^2 = 0
		H^2        + (W/2)^2 = 2*R*H
		(H^2       + (W/2)^2)/H = 2*R
	R =  (H^2      + (W/2)^2)/(H*2)
*/
(arc_h^2 + (arc_w/2)^2) / (arc_h*2);

/*****************************************************************************/
function ArcAngle( arc_h, arc_w, arc_r=undef, debug_prefix="" ) =
let(R=(is_undef(arc_r) ? ArcRadius(arc_h,arc_w,debug_prefix) : arc_r))
2*acos((R-arc_h)/R);

/*****************************************************************************/
// Return [ ArcAngle/2, radius ]
function ArcParams( arc_h, arc_w, arc_r=undef, debug_prefix="" ) =
let(ha=ArcAngle(arc_h,arc_w,arc_r,debug_prefix)/2)
[
	ha, (arc_w/2)/sin(ha)
];

/*****************************************************************************/
// Return [ floor( ArcAngle/2 ), radius ]
function ArcLoopParams( arc_h, arc_w, arc_r=undef, debug_prefix="" ) =
let(ha=floor(ArcAngle(arc_h,arc_w,arc_r,debug_prefix)/2))
[
	ha, (arc_w/2)/sin(ha)
];
