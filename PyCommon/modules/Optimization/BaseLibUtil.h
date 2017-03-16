#pragma once
#include "../../external_libraries/BaseLib/baselib.h"

inline numeric::array vectorn_2_pyVec(const vectorn& vecn)
{
	bp::list ls;
	for(int i=0; i<vecn.size(); ++i)
		ls.append(vecn[i]);
	return numeric::array(ls);
}