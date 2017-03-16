from fltk import *
import cPickle

import sys
if not '../PyCommon/modules/' in sys.path:
    sys.path.append('../PyCommon/modules/')
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


def test_adun_skeleton():
    bvhFilePath = './adun/ad.bvh'
    jointMotion = yf.readBvhFile(bvhFilePath)

    viewer = ysv.SimpleViewer()
    viewer.doc.addRenderer('adun', yr.JointMotionRenderer(jointMotion,(0,0,255), yr.LINK_LINE))
#    viewer.doc.addRenderer('adun', yr.JointMotionRenderer(jointMotion,(0,0,255), yr.POLYGON_LINE))
    viewer.doc.addObject('adun', jointMotion)

    viewer.startTimer(1/30.)
    viewer.show()

    Fl.run()

def test_adun_vp_skeleton():
    bvhFilePath = './adun/ad.bvh'
#    bvhFilePath = './adun/wd2.bvh'
    jointMotion = yf.readBvhFile(bvhFilePath)
    frameTime = 1/jointMotion.fps

    mcfgFilePath = './adun/mcfg'
#    mcfgFilePath = './ppmotion/mcfg'
    mcfgFile = open(mcfgFilePath, 'rb')
    mcfg = cPickle.load(mcfgFile)
    mcfgFile.close()

    node = mcfg.getNode("ground_pelvis")
#    node = mcfg.getNode("Hips")
    print node.localT
    print node.localR
    node.localT = (0,0,0)
    print node.localT

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
    test_adun_vp_skeleton()
