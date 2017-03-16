import cPickle
import sys
if '../PyCommon/modules/' not in sys.path:
    sys.path.append('../PyCommon/modules')
import Simulator.ysPhysConfig as ypc
import numpy.core.multiarray

if __name__=='__main__':
    ofile = open("./ppmotion/mcfg", 'rb')
    mcfg = cPickle.load(ofile)

    print mcfg

    print mcfg.getNode("Hips")
    print mcfg.getNode("Spine")
    print mcfg.getNode("Spine1")
    print mcfg.getNode("RightArm")
    print mcfg.getNode("LeftArm")
    print mcfg.getNode("RightFoot")
    print mcfg.getNode("LeftLeg")

