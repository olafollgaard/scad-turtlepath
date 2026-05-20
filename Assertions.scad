/*
	Assertions
*/

function AssertValidVectorList(
	// list to be validated, and returned if ok
	list,
	// expected length of each list value
	// - can be a simple number or a list of numbers
	expect_vector_size,
	// require each list value to have a nonzero norm (vector length)
	require_nonzero_norm=false,
	// prefix for assertion message
	prefix="") =
let(chk_evs=expect_vector_size)
assert(is_list(list) && len(list)>0 && is_list(list[0]), str(prefix,":list must be a nonempty list of vectors"))
assert(!is_num(chk_evs) || chk_evs > 0, str(prefix,":expected simple vector size must be positive"))
assert(!is_list(chk_evs) || len(AssertValidVectorList([chk_evs],len(chk_evs),prefix=prefix))==1)
assert(!is_list(chk_evs) || len(search(len(list[0]), chk_evs))>0,
	str(prefix,":expected vector sizes: ",chk_evs,", found: ",len(list[0])))
let(exp_sz= is_num(chk_evs) ? chk_evs : len(list[0]))
[
	for (i=[0:len(list)-1])
	let(item=list[i])
	assert(is_list(item), str(prefix,"@",i,":expected vector, found ",item))
	assert(len(item)==exp_sz,
		str(prefix,"@",i,":expected vector size ",exp_sz," found ",len(item),": ",item))
	let(check_item=
		[
			for(d=[0:exp_sz-1])
			assert(is_num(item[d]), str(prefix,"@",i,":expected numeric vector, found ",item))
			item[d]
		])
	assert(!require_nonzero_norm || norm(item)>0, str(prefix,"@",i,":expected nonzero vector"))
	item
];
