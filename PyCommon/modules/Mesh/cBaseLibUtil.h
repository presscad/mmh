#pragma once
#include "../../external_libraries/BaseLib/baselib.h"

#define PYSEQ_2_VECTOR3(seq)	(vector3(XD(seq[0]), XD(seq[1]), XD(seq[2])))

inline void PYT_FROM_MATRIX4(object& outT, matrix4& m4)
{
	outT[0][0] = m4.m[0][0]; outT[0][1] = m4.m[0][1]; outT[0][2] = m4.m[0][2]; outT[0][3] = m4.m[0][3]; 
	outT[1][0] = m4.m[1][0]; outT[1][1] = m4.m[1][1]; outT[1][2] = m4.m[1][2]; outT[1][3] = m4.m[1][3]; 
	outT[2][0] = m4.m[2][0]; outT[2][1] = m4.m[2][1]; outT[2][2] = m4.m[2][2]; outT[2][3] = m4.m[2][3]; 
	outT[3][0] = m4.m[3][0]; outT[3][1] = m4.m[3][1]; outT[3][2] = m4.m[3][2]; outT[3][3] = m4.m[3][3]; 
}