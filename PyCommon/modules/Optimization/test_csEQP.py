import psyco; psyco.full()
from fltk import *
import numpy as np
import numpy.linalg as npl

import sys
sys.path.append('..')
import Optimization.csEQP as ceq
import Math.mmMath as mm

def test_basic():
#    minimize :    f(x,y) = x^2 + y^2 + z^2
#    subject to :    x + y + z = 1
    p = ceq.EQP(3, 1)
    p.addSquaredTerm(1, (0,), (1., 0.))
    p.addSquaredTerm(1, (1,), (1., 0.))
    p.addSquaredTerm(1, (2,), (1., 0.))
    p.addConstraint((0,1,2), (1,1,1,-1))
    r = p.solve()
    print r

def test_constraint_num():
#    minimize :    f(x,y) = x^2 + y^2 + z^2

#    subject to :    x + y = 1
#                    y + z = 1
#                    x + z = 1
    p = ceq.EQP(3, 3)
    p.addSquaredTerm(1, (0,), (1., 0.))
    p.addSquaredTerm(1, (1,), (1., 0.))
    p.addSquaredTerm(1, (2,), (1., 0.))
    p.addConstraint((0,1), (1,1,-1))
    p.addConstraint((1,2), (1,1,-1))
    p.addConstraint((0,2), (1,1,-1))
    r = p.solve()
    print r
    
#    subject to :    x + y = 1
#                    y + z = 1
    p.clearConstraints()
    p.addConstraint((0,1), (1,1,-1))
    p.addConstraint((1,2), (1,1,-1))
    p.addConstraint((0,2), (0,0,-1))
    r = p.solve()
    print r

#    subject to :    x + y = 1
#                    y + z = 1
    p.clearConstraints()
    p.setConstraintNum(2)
    p.addSquaredTerm(1, (0,), (1., 0.))
    p.addSquaredTerm(1, (1,), (1., 0.))
    p.addSquaredTerm(1, (2,), (1., 0.))
    p.addConstraint((0,1), (1,1,-1))
    p.addConstraint((1,2), (1,1,-1))
    r = p.solve()
    print r

def test_constraint_rank():
#    subject to :    x + y = 1
#                    x + 2y = 0
#                    x + 3y = 0
    p = ceq.EQP(3, 3)
    p.addSquaredTerm(1, (0,), (1., 0.))
    p.addSquaredTerm(1, (1,), (1., 0.))
    p.addSquaredTerm(1, (2,), (1., 0.))
    p.addConstraint2((0,1), (1,1), -1)
    p.addConstraint2((0,1), (1,2), 0)
    p.addConstraint2((0,1), (1,3), 0)
    r = p.solve()
    print r
    
    p.clearConstraints()
    p.addConstraint2((0,1), (1,3), 0)
    p.addConstraint2((0,1), (1,1), -1)
    p.addConstraint2((0,1), (1,2), 0)
    r = p.solve()
    print r
    
    A = np.array([[1,1],
                  [1,2],
                  [1,3]])
    print mm.matrixrank(A)
    

def test_EQP3():
    p = ceq.EQP(6, 6)
    p.addSquaredTerm(1, (0,), (1., 0.))
    p.addSquaredTerm(1, (1,), (1., 0.))
    p.addSquaredTerm(1, (2,), (1., 9.7371072735721125))
    p.addSquaredTerm(1, (3,), (1., 0.))
    p.addSquaredTerm(1, (4,), (1., 0.))
    p.addSquaredTerm(1, (5,), (1., -9.7371072735721196))
    
    p.addConstraint2((0,1,2,3,4,5), [  1.0000e+00 ,  0.0000e+00 ,  0.0000e+00 ,  1.0000e+00 ,  0.0000e+00 ,  0.0000e+00] ,0.0)
    p.addConstraint2((0,1,2,3,4,5), [  0.0000e+00 ,  1.0000e+00 ,  0.0000e+00 ,  0.0000e+00 ,  1.0000e+00 ,  0.0000e+00] ,0.0)
    p.addConstraint2((0,1,2,3,4,5), [  0.0000e+00 ,  0.0000e+00 ,  1.0000e+00 ,  0.0000e+00 ,  0.0000e+00 ,  1.0000e+00] ,0.0)

    p.addConstraint2((0,1,2,3,4,5), [  0.0000e+00 ,  0.0000e+00 ,  1.0000e+00 ,  0.0000e+00 ,  0.0000e+00 ,  5.0000e-01], -14.142135623730951)
    p.addConstraint2((0,1,2,3,4,5), [  0.0000e+00 ,  0.0000e+00 ,  1.3450e-05 ,  0.0000e+00 ,  0.0000e+00 , -2.5080e-04] ,0.0)
    p.addConstraint2((0,1,2,3,4,5), [ -1.0000e+00 , -1.3450e-05 ,  0.0000e+00 , -5.0000e-01 ,  2.5080e-04 ,  0.0000e+00], 0.0)
    
    r = p.solve()
    print r
    
    p.clearConstraints()
    
    p.addConstraint2((0,1,2,3,4,5), [  0.0000e+00 ,  0.0000e+00 ,  1.0000e+00 ,  0.0000e+00 ,  0.0000e+00 ,  5.0000e-01], -14.142135623730951)
    p.addConstraint2((0,1,2,3,4,5), [  0.0000e+00 ,  0.0000e+00 ,  1.3450e-05 ,  0.0000e+00 ,  0.0000e+00 , -2.5080e-04] ,0.0)
    p.addConstraint2((0,1,2,3,4,5), [ -1.0000e+00 , -1.3450e-05 ,  0.0000e+00 , -5.0000e-01 ,  2.5080e-04 ,  0.0000e+00], 0.0)
    
    p.addConstraint2((0,1,2,3,4,5), [  1.0000e+00 ,  0.0000e+00 ,  0.0000e+00 ,  1.0000e+00 ,  0.0000e+00 ,  0.0000e+00] ,0.0)
    p.addConstraint2((0,1,2,3,4,5), [  0.0000e+00 ,  1.0000e+00 ,  0.0000e+00 ,  0.0000e+00 ,  1.0000e+00 ,  0.0000e+00] ,0.0)
    p.addConstraint2((0,1,2,3,4,5), [  0.0000e+00 ,  0.0000e+00 ,  1.0000e+00 ,  0.0000e+00 ,  0.0000e+00 ,  1.0000e+00] ,0.0)

    r = p.solve()
    print r
    
    A = np.array([[  0.0000e+00 ,  0.0000e+00 ,  1.0000e+00 ,  0.0000e+00 ,  0.0000e+00 ,  5.0000e-01],
                [  0.0000e+00 ,  0.0000e+00 ,  1.3450e-05 ,  0.0000e+00 ,  0.0000e+00 , -2.5080e-04] ,
                [ -1.0000e+00 , -1.3450e-05 ,  0.0000e+00 , -5.0000e-01 ,  2.5080e-04 ,  0.0000e+00], 
                [  1.0000e+00 ,  0.0000e+00 ,  0.0000e+00 ,  1.0000e+00 ,  0.0000e+00 ,  0.0000e+00] ,
                [  0.0000e+00 ,  1.0000e+00 ,  0.0000e+00 ,  0.0000e+00 ,  1.0000e+00 ,  0.0000e+00] ,
                [  0.0000e+00 ,  0.0000e+00 ,  1.0000e+00 ,  0.0000e+00 ,  0.0000e+00 ,  1.0000e+00]])
    print mm.matrixrank(A)
    print npl.det(A)
    
pass
#test_basic()
#test_constraint_num()
test_constraint_rank()
#test_EQP3()