import cPickle

import sys
if '../../PyCommon/modules' not in sys.path:
        sys.path.append('../../PyCommon/modules')
import Simulator.ysPhysConfig as ypc
import Resource.dcVRMLLoader as dvl
import numpy.core.multiarray
import renameJoint as rj
import preprocessMcfg as pm

def preprocessWrl(wrlFilePath, bvhSkelFilePath, mcfgFilePath):

    # read .wrl file
    wrl = dvl.readWrlFileasWrl(wrlFilePath)
    
    # rename by jointMap defined in renameJoint.py
    wrl = rj.renameBvhJoint(wrl)

    # write bvh skeleton
    bvh = wrl.toBvh()
    bvh.writeBvhFile(bvhSkelFilePath)

    # configure mcfg
    pm.configureMcfg(mcfgFilePath, wrl)

if __name__=='__main__':
    # make mcfg and bvhskel from wrl, renaming joints
    wrlFilePath = './osim/FullBody2_lee.wrl'
    mcfgFilePath = './mcfg'
    bvhSkelFilePath = './ad.bvh'
    preprocessWrl(wrlFilePath, bvhSkelFilePath, mcfgFilePath)
