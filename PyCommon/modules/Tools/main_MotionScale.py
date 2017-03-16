import psyco; psyco.full()
from fltk import *

import sys
sys.path.append('../PyCommon/modules')
import Math.mmMath as mmMath
import Resource.ysMotionLoader as yf
import Renderer.ysRenderer as yr
import GUI.ysSimpleViewer as ysv

if __name__ == "__main__":
    motion0, frameTime = yf.readBvhFileAsJointMotion('Data/wd2_WalkSameSame00.bvh', .01)
    motion1, frameTime = yf.readBvhFileAsJointMotion('Data/wd2_spiral_walk_normal05.bvh', .01)
    
    length0 = mmMath.length(motion0[0].getGlobalPos('RightUpLeg') - motion0[0].getGlobalPos('RightLeg'))
    length1 = mmMath.length(motion1[0].getGlobalPos('RightUpLeg') - motion1[0].getGlobalPos('RightLeg'))
    scale = length0 / length1
    print 'scale', scale
    
    scaledMotion1, frameTime = yf.readBvhFileAsJointMotion('Data/wd2_spiral_walk_normal05.bvh', .01*scale)
    
    viewer = ysv.SimpleViewer()
    viewer.record(False)
    viewer.doc.addRenderer('motion0', yr.JointMotionRenderer(motion0, (255,0,0), yr.LINK_BONE))
    viewer.doc.addRenderer('motion1', yr.JointMotionRenderer(motion1, (0,0,255), yr.LINK_BONE))
    viewer.doc.addRenderer('scaledMotion1', yr.JointMotionRenderer(scaledMotion1, (0,255,0), yr.LINK_BONE))
    
    viewer.startTimer(frameTime)
    viewer.show()
    
    Fl.run()