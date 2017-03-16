import psyco; psyco.full()
import numpy as np

import sys
sys.path.append('..')
import Math.mmMath as mm
import Math.csMath as cm

def test_zyx():
    x, y, z = (1.,.5,2.)
    print 'orig ', x, y, z
    
    R = mm.I_SO3()
    R = np.dot(R, mm.rotZ(z))
    R = np.dot(R, mm.rotX(x))
    R = np.dot(R, mm.rotY(y))
    nz, nx, ny = cm.R2zxy_r(R)
    print 'zxy_r', nx, ny, nz
    
    R = mm.I_SO3()
    R = np.dot(R, mm.rotY(y))
    R = np.dot(R, mm.rotX(x))
    R = np.dot(R, mm.rotZ(z))
    nz, nx, ny = cm.R2zxy_s(R)
    print 'zxy_s', nx, ny, nz
    
    R = mm.I_SO3()
    R = np.dot(R, mm.rotX(x))
    R = np.dot(R, mm.rotY(y))
    R = np.dot(R, mm.rotZ(z))
    nx, ny, nz = cm.R2xyz_r(R)
    print 'xyz_r', nx, ny, nz

test_zyx()