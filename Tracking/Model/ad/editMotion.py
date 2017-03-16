from fltk import *
import cPickle
import numpy as np

import sys
if not '../../../PyCommon/modules/' in sys.path:
    sys.path.append('../../../PyCommon/modules/')
import Resource.ysMotionLoader as yf
import GUI.ysSimpleViewer as ysv
import Renderer.ysRenderer as yr
import Renderer.csVpRenderer as cvr
import Simulator.csVpWorld as cvw
import Simulator.csVpModel as cvm
import Simulator.ysVpUtil as yvu
import Simulator.ysPhysConfig as ypc
import Motion.ysMotionConverter as ymc
import Motion.ysMotion as ym
import Motion.mmAnalyticIK as mai
import Util.ysGlHelper as ygh
import Util.ysPythonEx as ype
import Util.dcFileIO as df
import Motion.ysSkeletonEdit as yme
import Math.mmMath as mm
import math

def editMotion():

    # motion
    dir = './'
    motionDir = './motion/'
    modelName = 'ad'
    motionName = 'WalkForwardNormal00'
    motionFileName = modelName+'_'+motionName+'.bvh'

    motion = yf.readBvhFile(motionDir+motionFileName)
    prev_motion = motion.copy()

    editRoot(motion)

    bvh = yf.Bvh()
    bvh.fromJointMotion(motion)
    bvh.writeBvhFile(motionDir+motionFileName)

    motion = yf.readBvhFile(motionDir+motionFileName)

    showMotion([prev_motion, motion], ["prev_motion", "motion"], [(255,255,255),(0,255,0)])

def editRoot(motion):

    for i in range(len(motion)):
        moveRootPosY = 0.0#-0.01
        
        rootPos = motion[i].getRootPos()
        rootPos[1] = rootPos[1] + moveRootPosY
        motion[i].setRootPos(rootPos)
    
def showMotion(motions, names=None, colors=None):

    viewer = ysv.SimpleViewer()
    viewer.initialize()
    viewer.record(False)
    default_color = (255,255,255)
    default_name = 'motion'
    for i in range(len(motions)):
        if names != None:
            name = names[i]
        else:
            name = default_name + str(i)
        if colors != None:
            color = colors[i]
        else:
            color = default_color
        viewer.doc.addRenderer(name, yr.JointMotionRenderer(motions[i],color, yr.POLYGON_LINE))
        viewer.doc.addObject(name, motions[i])

    viewer.startTimer(1/motions[0].fps)
    viewer.show()

    Fl.run()

if __name__=='__main__':
    editMotion()
