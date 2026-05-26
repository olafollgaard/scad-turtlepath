/*
	3d vector projection
*/
use <Assertions.scad>;

function Project3dVector(v,k,debug_prefix="") =
// projB(A) = (A . B / |B|^2) * B
let(check_v=AssertValidVectorList([v],3,require_nonzero_norm=false,prefix=debug_prefix),
	check_k=AssertValidVectorList([k],3,require_nonzero_norm=true,prefix=debug_prefix),
	dotVK=v*k,
	sqrK=k*k,
	factor=dotVK/sqrK)
factor * k;

function VectorFrom3dPointToPlane(pt,pnv,ppt,debug_prefix="") =
let(check_pt=AssertValidVectorList([pt],3,require_nonzero_norm=false,prefix=debug_prefix),
	check_pnv=AssertValidVectorList([pnv],3,require_nonzero_norm=true,prefix=debug_prefix),
	check_ppt=AssertValidVectorList([ppt],3,require_nonzero_norm=false,prefix=debug_prefix))
Project3dVector(ppt-pt,pnv,debug_prefix=debug_prefix);

function DistanceFrom3dPointToPlane(pt,pnv,ppt,debug_prefix) =
let(vDiff=VectorFrom3dPointToPlane(pt,pnv,ppt,debug_prefix))
norm(vDiff);

function Project3dPointToPlane(pt,pnv,ppt,debug_prefix="") =
let(vDiff=VectorFrom3dPointToPlane(pt,pnv,ppt,debug_prefix))
pt+vDiff;
