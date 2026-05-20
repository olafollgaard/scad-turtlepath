/*
	Vector rotation using quaternions
	https://en.wikipedia.org/wiki/Quaternion
*/

function Rotation3dAxis_Quaternion(k,a,debug_prefix="") =
assert(is_list(k) && len(k)==3 && norm(k)>0,debug_prefix)
assert(is_num(a),debug_prefix)
let(uk=k/norm(k),
	ha=a/2,
	ca=cos(ha),
	sa=sin(ha))
[ ca, uk.x*sa, uk.y*sa, uk.z*sa ];

function Rotate3dVector_UseQuaternion(v,q,debug_prefix="") =
assert(is_list(v) && len(v)==3,debug_prefix)
let(p=[0,v.x,v.y,v.z],
	temp=qt_Multiply(q,p),
	q_conj=qt_Conjugate(q),
	p_prime=qt_Multiply(temp,q_conj))
[ p_prime[1], p_prime[2], p_prime[3] ];

function Rotate3dVector(v,k,a,debug_prefix="") =
Rotate3dVector_UseQuaternion(v,Rotation3dAxis_Quaternion(k,a,debug_prefix),debug_prefix);

function qt_Multiply(q1,q2) =
let(w1=q1[0],x1=q1[1],y1=q1[2],z1=q1[3],
	w2=q2[0],x2=q2[1],y2=q2[2],z2=q2[3],
	w = w1*w2 - x1*x2 - y1*y2 - z1*z2,
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2,
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2,
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2)
[ w, x, y, z ];

function qt_Conjugate(q) =
[ q[0], -q[1], -q[2], -q[3] ];
