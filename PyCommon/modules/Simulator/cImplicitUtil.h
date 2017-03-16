#pragma once

#define PYSEQ_2_VECTOR3(seq)	(Physics_Vector3(XD(seq[0]), XD(seq[1]), XD(seq[2])))
#define VECTOR3_2_PYTUPLE(vec3) (make_tuple(vec3.x, vec3.y, vec3.z))

#define FLT_EPSILON     1.192092896e-07F        /* smallest such that 1.0+FLT_EPSILON != 1.0 */
#define DBL_EPSILON     2.2204460492503131e-016 /* smallest such that 1.0+DBL_EPSILON != 1.0 */
