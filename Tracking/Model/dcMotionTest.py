from fltk import *
import cPickle
import numpy as np

import sys
if not '../../PyCommon/modules/' in sys.path:
    sys.path.append('../../PyCommon/modules/')
import Resource.ysMotionLoader as yf
import GUI.ysSimpleViewer as ysv
import Renderer.ysRenderer as yr
import Renderer.csVpRenderer as cvr
import Simulator.csVpWorld as cvw
import Simulator.csVpModel as cvm
import Simulator.ysVpUtil as yvu
import Simulator.ysPhysConfig as ypc
import Motion.ysMotionConverter as ymc
import Util.ysGlHelper as ygh
import Util.ysPythonEx as ype
import Motion.ysSkeletonEdit as yme
import Math.mmMath as mm
import math

def measure_ad_wd_skeleton():
    adFilePath = './ad/ad.bvh'
    wdFilePath = './wd2/wd2.bvh'
    wdFilePath = './wd2/wd2_WalkForwardNormal00.bvh'
    adJointMotion = yf.readBvhFile(adFilePath)
    wdJointMotion = yf.readBvhFile(wdFilePath)

    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.0
    wcfg.useDefaultContactModel = False
    stepsPerFrame = 30
    wcfg.timeStep = (1/wdJointMotion.fps)/stepsPerFrame

    adMcfgFilePath = './ad/mcfg'
    wdMcfgFilePath = './wd2/mcfg'
    adMcfgFile = open(adMcfgFilePath, 'rb')
    adMcfg = cPickle.load(adMcfgFile)
    adMcfgFile.close()
    wdMcfgFile = open(wdMcfgFilePath, 'rb')
    wdMcfg = cPickle.load(wdMcfgFile)
    wdMcfgFile.close()

    adJointMotion[0].setPosition(0, wdJointMotion[0].getPosition(0))
    
    vpWorld = cvw.VpWorld(wcfg)
#    adMotionModel = cvm.VpMotionModel(vpWorld, adJointMotion[0].getTPose(), adMcfg)
#    wdMotionModel = cvm.VpMotionModel(vpWorld, wdJointMotion[0].getTPose(), wdMcfg)
    adModel = cvm.VpControlModel(vpWorld, adJointMotion[0].getTPose(), adMcfg)
    wdModel = cvm.VpControlModel(vpWorld, wdJointMotion[0].getTPose(), wdMcfg)
#    vpWorld.initialize()

    ankle_r_idx = adModel.name2index("ankle_r")
    ankle_l_idx = adModel.name2index("ankle_l")
    hip_l_idx = adModel.name2index("hip_l")
    LeftFoot_idx = wdModel.name2index("LeftFoot")
    LeftUpLeg_idx = wdModel.name2index("LeftUpLeg")
#    a = adModel.getBodyPositionGlobal(adModel.name2index("ankle_r"))
#    print a[0], a[1], a[2]

#   wd
    local_zero = wdModel.getBodyPositionGlobal(LeftFoot_idx, (0,0,0))
    local_x_axis = wdModel.getBodyPositionGlobal(LeftFoot_idx, (1,0,0))-local_zero
    local_y_axis = wdModel.getBodyPositionGlobal(LeftFoot_idx, (0,1,0))-local_zero
    local_z_axis = wdModel.getBodyPositionGlobal(LeftFoot_idx, (0,0,1))-local_zero
    local_Transform = np.transpose(np.array([local_x_axis/np.linalg.norm(local_x_axis), local_y_axis/np.linalg.norm(local_y_axis), local_z_axis/np.linalg.norm(local_z_axis)], float))
#    print local_Transform

    wdTPose = wdJointMotion[0].getTPose()
    wdTPose.mulGlobalR(wdTPose.skeleton.getElementIndex("LeftFoot"), local_Transform.T)
    wdTPose.mulGlobalR(wdTPose.skeleton.getElementIndex("RightFoot"), local_Transform.T)
    
    wdModel2 = cvm.VpControlModel(vpWorld, wdTPose, wdMcfg)
    
    # wd2
    leftFootNode = wdMcfg.getNode("LeftFoot")
    wd2_footHeight = math.fabs(leftFootNode.mass / (leftFootNode.density * leftFootNode.length * leftFootNode.width * 2.0))
    wd2_ankleHeight = wdModel2.getJointPositionGlobal(LeftFoot_idx)[1] - wdModel2.getBodyPositionGlobal(LeftFoot_idx, (0,-wd2_footHeight,0))[1]
    print "wd2_ankleHeight\t", wd2_ankleHeight
   
    wd2_legVec = wdModel2.getBodyPositionGlobal(LeftUpLeg_idx) - wdModel2.getBodyPositionGlobal(LeftFoot_idx)
    print "wd2_legVec\t", wd2_legVec
    print "wd2_legLength\t", np.linalg.norm(wd2_legVec)

    # ad
    ankle_l_node = adMcfg.getNode("ankle_l")
    ad_footHeight = math.fabs(ankle_l_node.mass / (ankle_l_node.density * ankle_l_node.length * ankle_l_node.width * 2.0))
    ad_ankleHeight = adModel.getJointPositionGlobal(ankle_l_idx)[1] - adModel.getBodyPositionGlobal(ankle_l_idx, (0,-ad_footHeight,0))[1]
    print "ad_ankleHeight\t", ad_ankleHeight

    ad_legVec = adModel.getBodyPositionGlobal(hip_l_idx) - adModel.getBodyPositionGlobal(ankle_l_idx)
    print "ad_legVec\t", ad_legVec
    print "ad_legLength\t", np.linalg.norm(ad_legVec)



    vpWorld.initialize()
    viewer = ysv.SimpleViewer()
    viewer.doc.addRenderer('ad', yr.JointMotionRenderer(adJointMotion,(0,0,255), yr.POLYGON_LINE))
    viewer.doc.addRenderer('wd', yr.JointMotionRenderer(wdJointMotion,(0,255,0), yr.POLYGON_LINE))
    viewer.doc.addRenderer('adVpModel', cvr.VpModelRenderer(adModel, (200,200,200), yr.POLYGON_LINE))
    viewer.doc.addRenderer('wdVpModel', cvr.VpModelRenderer(wdModel2, (0,255,0), yr.POLYGON_LINE))
    viewer.startTimer(1/30.)
    viewer.show()

    Fl.run()

def compare_ad_wd_skeleton():
    adFilePath = './ad/ad.bvh'
    wdFilePath = './wd2/wd2.bvh'
    wdFilePath = './wd2/wd2_WalkForwardNormal00.bvh'
    adJointMotion = yf.readBvhFile(adFilePath)
    wdJointMotion = yf.readBvhFile(wdFilePath)

    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.0
    wcfg.useDefaultContactModel = False
    stepsPerFrame = 30
    wcfg.timeStep = (1/wdJointMotion.fps)/stepsPerFrame

    adMcfgFilePath = './ad/mcfg'
    wdMcfgFilePath = './wd2/mcfg'
    adMcfgFile = open(adMcfgFilePath, 'rb')
    adMcfg = cPickle.load(adMcfgFile)
    adMcfgFile.close()
    wdMcfgFile = open(wdMcfgFilePath, 'rb')
    wdMcfg = cPickle.load(wdMcfgFile)
    wdMcfgFile.close()

    vpWorld = cvw.VpWorld(wcfg)
    adModel = cvm.VpMotionModel(vpWorld, adJointMotion[0].getTPose(), adMcfg)
    wdModel = cvm.VpMotionModel(vpWorld, wdJointMotion[0].getTPose(), wdMcfg)
    vpWorld.initialize()

    viewer = ysv.SimpleViewer()
#    viewer.doc.addRenderer('ad', yr.JointMotionRenderer(adJointMotion,(0,0,255), yr.POLYGON_LINE))
#    viewer.doc.addRenderer('wd', yr.JointMotionRenderer(wdJointMotion,(0,255,0), yr.POLYGON_LINE))
    viewer.doc.addRenderer('adVpModel', cvr.VpModelRenderer(adModel, (200,200,200), yr.POLYGON_LINE))
    viewer.doc.addRenderer('wdVpModel', cvr.VpModelRenderer(wdModel, (0,255,0), yr.POLYGON_LINE))
    viewer.startTimer(1/30.)
    viewer.show()

    Fl.run()


def test_adun_skeleton():
    bvhFilePath = './ad/ad.bvh'
    jointMotion = yf.readBvhFile(bvhFilePath)

    viewer = ysv.SimpleViewer()
    viewer.doc.addRenderer('adun', yr.JointMotionRenderer(jointMotion,(0,0,255), yr.LINK_LINE))
#    viewer.doc.addRenderer('adun', yr.JointMotionRenderer(jointMotion,(0,0,255), yr.POLYGON_LINE))
    viewer.doc.addObject('adun', jointMotion)

    viewer.startTimer(1/30.)
    viewer.show()

    Fl.run()

def test_adun_vp_skeleton():
    bvhFilePath = './ad/ad.bvh'
#    bvhFilePath = './adun/wd2.bvh'
    jointMotion = yf.readBvhFile(bvhFilePath)
    frameTime = 1/jointMotion.fps

    mcfgFilePath = './ad/mcfg'
#    mcfgFilePath = './ppmotion/mcfg'
    mcfgFile = open(mcfgFilePath, 'rb')
    mcfg = cPickle.load(mcfgFile)
    mcfgFile.close()

    node = mcfg.getNode("ground_pelvis")
#    node = mcfg.getNode("Hips")

#    node = mcfg.getNode("ankle_l")
#    node.localT = (0,0,0)
#    node.offset = (0,0,0)
#    node.localR = mm.I_SO3()
    
#    node = mcfg.getNode("ankle_r")

    print jointMotion[0].skeleton
    print jointMotion[0].skeleton.root

    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
#    wcfg.lockingVel = c_locking_vel
    stepsPerFrame = 30
    wcfg.timeStep = (frameTime)/stepsPerFrame

    vpWorld = cvw.VpWorld(wcfg)
    motionModel = cvm.VpMotionModel(vpWorld, jointMotion[0], mcfg)
    vpWorld.initialize()

#    print jointMotion[0].skeleton.getJointIndex('RightFoot')
#    print motionModel.name2index('RightFoot')
#    print len(jointMotion[0].localRs)
#    print jointMotion[0].getJointOrientationLocal(motionModel.name2index('RightFoot'))
#    print motionModel.getBodyNum()
#    print motionModel.name2index("LeftLeg")

    viewer = ysv.SimpleViewer()
    viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, (200,200,200), yr.POLYGON_LINE))

    viewer.startTimer(1/frameTime)
    viewer.show()

    Fl.run()


if __name__=='__main__':
    #test_adun_skeleton()
    #test_adun_vp_skeleton()
    #compare_ad_wd_skeleton()
    measure_ad_wd_skeleton()
