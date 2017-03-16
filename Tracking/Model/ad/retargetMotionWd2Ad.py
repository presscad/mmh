from fltk import *
import cPickle
import numpy as np

import sys
if '../../../PyCommon/modules' not in sys.path:
    sys.path.append('../../../PyCommon/modules')
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

# retarget Motion from wd2 to ad
#                  s(source)  t(target)
def retargetMotion():

    # target
    tDir = './'
    tMotionDir = './Motion/'
    tSkeletonFileName = 'ad.bvh'
    tModelName = 'ad'

    # source
    sDir = '../wd2/'
    sMotionDir = '../wd2/Motion/'
    sMotionFileName = 'wd2_WalkForwardNormal00.bvh'
    sMotionFileName = 'wd2_WalkAzuma01.bvh'

    # TMotion = [TPose]
    # tMotion : target motion, sMotion : source motion
    tTMotion = yf.readBvhFile(tDir + tSkeletonFileName)
    tTMotion[0] = tTMotion[0].getTPose()
    sMotion = yf.readBvhFile(sMotionDir + sMotionFileName)
    tMotion = ym.JointMotion([tTMotion[0].copy() for i in range(len(sMotion))])
    tMotion.fps = sMotion.fps
    tMotion.frame = sMotion.frame

    # calc ankle height, leg length, upperLegFromRoot
    sAnkleHeight, sLegLength, tAnkleHeight, tLegLength = measure_skeleton()    
    print sAnkleHeight, sLegLength, tAnkleHeight, tLegLength
    legScale = tLegLength/sLegLength
    print legScale

    rootDiffR = mm.rotY(-math.pi/2)
    retargetRoot(sMotion, tMotion, rootDiffR, sAnkleHeight, tAnkleHeight, legScale)
    retargetBack(sMotion, tMotion)
    retargetArm(sMotion, tMotion, rootDiffR)

    retargetLeg(sMotion, tMotion)
    retargetFoot(sMotion, tMotion)

    _, motionFileName = sMotionFileName.split('_', 1)
    tMotionFileName = tModelName+'_'+motionFileName
    bvh = yf.Bvh()
    bvh.fromJointMotion(tMotion)
    bvh.writeBvhFile(tMotionDir+tMotionFileName)

    rMotion = yf.readBvhFile(tMotionDir+tMotionFileName)

    #showMotion([sMotion, tMotion], ["wd2", "ad"], [(255,255,255),(0,255,0)])
    showMotion([sMotion, tMotion, rMotion], ["wd2", "ad", "written_ad"], [(255,255,255),(0,255,0),(255,255,0)])


def retargetRoot(sMotion, tMotion, rootDiffR, sAnkleHeight, tAnkleHeight, legScale):
    
    sMotionUpLegs = ['LeftUpLeg', 'RightUpLeg']
    tMotionUpLegs = ['hip_l', 'hip_r']
    sMotionUpLegs_idx = [sMotion[0].skeleton.getJointIndex(f) for f in sMotionUpLegs]
    tMotionUpLegs_idx = [tMotion[0].skeleton.getJointIndex(f) for f in tMotionUpLegs]
    
    # to edit manually
    rootPosOffset = np.array([0.0, -0.01, 0.0],float)

    for i in range(len(sMotion)):
        # sRootR * rootDiffR * tR
        # tMotion[i].mulJointOrientationGlobal(0, rootDiffR)
        # tMotion[i].setJointOrientationLocal(0, np.dot(sMotion[i].getJointOrientationLocal(0), tMotion[i].getJointOrientationLocal(0)))
        tMotion[i].mulJointOrientationGlobal(0, rootDiffR)
        tMotion[i].mulJointOrientationGlobal(0, sMotion[i].getJointOrientationLocal(0))
        tMotion[i].updateGlobalT()
        
        sUpperLegPos = (sMotion[i].getJointPositionGlobal(sMotionUpLegs_idx[0]) + sMotion[i].getJointPositionGlobal(sMotionUpLegs_idx[1])) / 2.0
        sUpperLegPos[1] = (sUpperLegPos[1] - sAnkleHeight) * legScale + tAnkleHeight
        tUpperLegPosFromRoot = (tMotion[i].getJointPositionGlobal(tMotionUpLegs_idx[0]) + tMotion[i].getJointPositionGlobal(tMotionUpLegs_idx[1])) / 2.0 - tMotion[i].getJointPositionGlobal(0)
        
        tMotion[i].setRootPos(sUpperLegPos - tUpperLegPosFromRoot + rootPosOffset)
        tMotion[i].updateGlobalT()
    
def copyJointOrientations(sPosture, tPosture, jointNames):
    jointIdxs = [tPosture.skeleton.getJointIndex(f) for f in jointNames]
    for i in range(len(jointNames)):
        tPosture.setJointOrientationLocal(jointIdxs[i], sPosture.getJointOrientationLocal(jointIdxs[i]))
    tPosture.updateGlobalT()

def retargetLeg(sMotion, tMotion):
   
    sMotionFeet = ['LeftFoot', 'RightFoot']
    tMotionFeet = ['ankle_l', 'ankle_r']
    sMotionFeet_idx = [sMotion[0].skeleton.getJointIndex(f) for f in sMotionFeet]
    tMotionFeet_idx = [tMotion[0].skeleton.getJointIndex(f) for f in tMotionFeet]

    sMotionLegs = ['LeftLeg', 'RightLeg']
    tMotionLegs = ['knee_l', 'knee_r']
    sMotionLegs_idx = [sMotion[0].skeleton.getJointIndex(f) for f in sMotionLegs]
    tMotionLegs_idx = [tMotion[0].skeleton.getJointIndex(f) for f in tMotionLegs]

    tMotionUpLegs = ['hip_l', 'hip_r']
    tMotionUpLegs_idx = [tMotion[0].skeleton.getJointIndex(f) for f in tMotionUpLegs]
    
    threshold = 0.01

    # default
    footPosOffset = np.array([0.,0.,0.],float)
    kneePosOffset = np.array([0.,0.,0.],float)

    # original
    #footPosOffset = np.array([0.068,0.0,0.06],float)
    #footPosOffset = np.array([0.064,0.0,0.03],float)
    footPosOffset = np.array([0.055,0.0,0.0],float)
    #kneePosOffset = np.array([0.0, 0.0, 0.0], float)

    # azuma_normal
    footPosOffset = np.array([0.055,0.0,0.0],float)

    # upperBody mass 1/10 modification
    #footPosOffset = np.array([0.065,0.0,0.0],float)
    #kneePosOffset = np.array([0.0, 0.0, 0.0], float)


    for i in range(len(sMotion)):
        if i>0:
            copyJointOrientations(tMotion[i-1], tMotion[i], ['hip_l', 'knee_l', 'ankle_l', 'mtp_l', 'hip_r', 'knee_r', 'ankle_r', 'mtp_r'])
        
        for j in range(2):
            
            ikFootPosOffset = np.copy(footPosOffset)
            ikKneePosOffset = np.copy(kneePosOffset)
            if j==1:
                ikFootPosOffset[2] = -ikFootPosOffset[2]
                ikKneePosOffset[2] = -ikKneePosOffset[2]
            
            mai.ik_analytic(tMotion[i], tMotionFeet[j], sMotion[i].getJointPositionGlobal(sMotionFeet_idx[j])+ikFootPosOffset)
            
            sLegPos = sMotion[i].getJointPositionGlobal(sMotionLegs_idx[j])
            tLegPos = tMotion[i].getJointPositionGlobal(tMotionLegs_idx[j])
            tFootPos = tMotion[i].getJointPositionGlobal(tMotionFeet_idx[j])
            tUpLegPos = tMotion[i].getJointPositionGlobal(tMotionUpLegs_idx[j])
            if ikKneePosOffset != None:
                sLegPos += ikKneePosOffset

            sLegVec = sLegPos - tUpLegPos
            tLegVec = tLegPos - tUpLegPos
            tFootVec = tFootPos - tUpLegPos

            # residual vector of (LegPos - UpperLegPos)
            sLegProjVec, sLegResVec = mm.projectionOnVector2(sLegVec, tFootVec)
            tLegProjVec, tLegResVec = mm.projectionOnVector2(tLegVec, tFootVec)
           
            if mm.length(sLegResVec) > threshold and mm.length(tLegResVec) > threshold:
                
                if np.inner(tLegResVec, sLegResVec) < 0.0:
                    # reverse leg orientation
                    tFootVec_n = mm.normalize(tFootVec)
                    tLegVec_n = mm.normalize(tLegVec)

                    rotV = np.cross(tLegVec_n, tFootVec_n)
                    rotA = mm.ACOS(np.inner(tLegVec_n, tFootVec_n)) * 2.0
                    tMotion[i].mulJointOrientationGlobal(tMotionUpLegs_idx[j], mm.exp(rotV, rotA))

                    tFootLegVec = tFootPos - tLegPos
                    tFootLegVec_n = mm.normalize(tFootLegVec)

                    rotV = np.cross(tFootLegVec_n, tLegVec_n)
                    rotA = mm.ACOS(np.inner(tFootLegVec_n, tLegVec_n)) * 2.0
                    tMotion[i].mulJointOrientationGlobal(tMotionLegs_idx[j], mm.exp(rotV, rotA))

                    tLegResVec = -tLegResVec
                
                tFootVec_n = mm.normalize(tFootVec)
                sLegResVec_n = mm.normalize(sLegResVec)
                tLegResVec_n = mm.normalize(tLegResVec)

                sLegT = np.transpose(np.array([sLegResVec_n, tFootVec_n, np.cross(sLegResVec_n,tFootVec_n)],float))
                tLegT = np.transpose(np.array([tLegResVec_n, tFootVec_n, np.cross(tLegResVec_n,tFootVec_n)],float))

                # setJointOrientationLocal includes updateGlobalT
                tMotion[i].mulJointOrientationGlobal(tMotionUpLegs_idx[j], np.dot(sLegT, tLegT.T))

def retargetFoot(sMotion, tMotion):

    sMotionFeet = ['LeftFoot', 'RightFoot']
    tMotionFeet = ['ankle_l', 'ankle_r']
    sMotionFeet_idx = [sMotion[0].skeleton.getJointIndex(f) for f in sMotionFeet]
    tMotionFeet_idx = [tMotion[0].skeleton.getJointIndex(f) for f in tMotionFeet]

    # foot_end is a element, not a joint
    sMotionFeetEnd = [f+"_Effector" for f in sMotionFeet]
    sMotionFeetEnd_idx = [sMotion[0].skeleton.getElementIndex(f) for f in sMotionFeetEnd]

    # tMotion local foot direction at tpose()
    tFootLocalDir = np.array([1.0,0.0,0.0],float)

    for i in range(len(sMotion)):
        
        for j in range(2):
            # sMotion foot direction
            # foot_end is a element, not a joint
            sFootDir = sMotion[i].getPosition(sMotionFeetEnd_idx[j]) - sMotion[i].getJointPositionGlobal(sMotionFeet_idx[j])

            # tMotion foot direction
            tFootDir = np.dot(tMotion[i].getJointOrientationGlobal(tMotionFeet_idx[j]), tFootLocalDir)

            sFootDir_n = mm.normalize(sFootDir)
            tFootDir_n = mm.normalize(tFootDir)

            rotV = np.cross(tFootDir_n, sFootDir_n)
            rotA = mm.ACOS(np.inner(tFootDir_n, sFootDir_n))

            if mm.length(rotV) > 0:
                tMotion[i].mulJointOrientationGlobal(tMotionFeet_idx[j], mm.exp(rotV, rotA))

# the retargetBack function follows the retargetRoot function
def retargetBack(sMotion, tMotion):

    sMotionBack = ['Spine', 'Spine1']
    tMotionBack = ['back']
    sMotionBack_idx = [sMotion[0].skeleton.getJointIndex(f) for f in sMotionBack]
    tMotionBack_idx = [tMotion[0].skeleton.getJointIndex(f) for f in tMotionBack]

    sMotionArms = ['LeftArm', 'RightArm']
    tMotionArms = ['shoulder_l', 'shoulder_r']
    sMotionArms_idx = [sMotion[0].skeleton.getJointIndex(f) for f in sMotionArms]
    tMotionArms_idx = [tMotion[0].skeleton.getJointIndex(f) for f in tMotionArms]

    for i in range(len(sMotion)):
        sCenterOfArms = (sMotion[i].getJointPositionGlobal(sMotionArms_idx[0]) + sMotion[i].getJointPositionGlobal(sMotionArms_idx[1])) / 2.0
        sLeftArmVec_n = mm.normalize(sMotion[i].getJointPositionGlobal(sMotionArms_idx[0]) - sCenterOfArms)
        sBackVec_n = mm.normalize(sMotion[i].getJointPositionGlobal(sMotionBack_idx[0]) - sCenterOfArms)
        sCrossAxis_n = mm.normalize(np.cross(sLeftArmVec_n, sBackVec_n))
        
        tCenterOfArms = (tMotion[i].getJointPositionGlobal(tMotionArms_idx[0]) + tMotion[i].getJointPositionGlobal(tMotionArms_idx[1])) / 2.0
        tLeftArmVec_n = mm.normalize(tMotion[i].getJointPositionGlobal(tMotionArms_idx[0]) - tCenterOfArms)
        tBackVec_n = mm.normalize(tMotion[i].getJointPositionGlobal(tMotionBack_idx[0]) - tCenterOfArms)
        tCrossAxis_n = mm.normalize(np.cross(tLeftArmVec_n, tBackVec_n))

        sBackT = np.transpose(np.array([sLeftArmVec_n,np.cross(sCrossAxis_n,sLeftArmVec_n),sCrossAxis_n],float))
        tBackT = np.transpose(np.array([tLeftArmVec_n,np.cross(tCrossAxis_n,tLeftArmVec_n),tCrossAxis_n],float))
        tMotion[i].mulJointOrientationGlobal(tMotionBack_idx[0], np.dot(sBackT, tBackT.T))

def retargetArm(sMotion, tMotion, rootDiffR):
    # without ik
    # like the method used to retarget foot
    
    sArms = ['LeftArm', 'RightArm']
    tArms = ['shoulder_l', 'shoulder_r']
    sArms_idx = [sMotion[0].skeleton.getJointIndex(f) for f in sArms]
    tArms_idx = [tMotion[0].skeleton.getJointIndex(f) for f in tArms]

    sForeArms = ['LeftForeArm', 'RightForeArm']
    tForeArms = ['radioulnar_l', 'radioulnar_r']
    sForeArms_idx = [sMotion[0].skeleton.getJointIndex(f) for f in sForeArms]
    tForeArms_idx = [tMotion[0].skeleton.getJointIndex(f) for f in tForeArms]

    # wd2 model use an End_Effector of forearm as hand, so I use an element index for hand
    sHands = [f+'_Effector' for f in sForeArms]
    tHands = ['radius_hand_l', 'radius_hand_r']
    sHands_eidx = [sMotion[0].skeleton.getElementIndex(f) for f in sHands]
    tHands_eidx = [tMotion[0].skeleton.getElementIndex(f) for f in tHands]

    for i in range(len(sMotion)):

        for j in range(2):
            # Arm
            sForeArmVec_n = mm.normalize(sMotion[i].getJointPositionGlobal(sForeArms_idx[j]) - sMotion[i].getJointPositionGlobal(sArms_idx[j]))
            tForeArmVec_n = mm.normalize(tMotion[i].getJointPositionGlobal(tForeArms_idx[j]) - tMotion[i].getJointPositionGlobal(tArms_idx[j]))

            rotV = np.cross(tForeArmVec_n, sForeArmVec_n)
            rotA = mm.ACOS(np.inner(tForeArmVec_n, sForeArmVec_n))

            if mm.length(rotV) > 0.0:
                tMotion[i].mulJointOrientationGlobal(tArms_idx[j], mm.exp(rotV, rotA))
            
            # forearm
            sHandVec_n = mm.normalize(sMotion[i].getPosition(sHands_eidx[j]) - sMotion[i].getJointPositionGlobal(sForeArms_idx[j]))
            tHandVec_n = mm.normalize(tMotion[i].getPosition(tHands_eidx[j]) - tMotion[i].getJointPositionGlobal(tForeArms_idx[j]))

            rotV = np.cross(tHandVec_n, sHandVec_n)
            rotA = mm.ACOS(np.inner(tHandVec_n, sHandVec_n))

            if mm.length(rotV) > 0.0:
                tMotion[i].mulJointOrientationGlobal(tForeArms_idx[j], mm.exp(rotV, rotA))




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


def measure_skeleton():
    adFilePath = './ad.bvh'
    wdFilePath = '../wd2/wd2.bvh'
    adJointMotion = yf.readBvhFile(adFilePath)
    wdJointMotion = yf.readBvhFile(wdFilePath)

    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.0
    wcfg.useDefaultContactModel = False
    stepsPerFrame = 30
    wcfg.timeStep = (1/wdJointMotion.fps)/stepsPerFrame

    adMcfgFilePath = './mcfg'
    wdMcfgFilePath = '../wd2/mcfg'
    adMcfg = df.readPickle(adMcfgFilePath)
    wdMcfg = df.readPickle(wdMcfgFilePath)

    adJointMotion[0].setPosition(0, wdJointMotion[0].getPosition(0))
    
    vpWorld = cvw.VpWorld(wcfg)
    adModel = cvm.VpControlModel(vpWorld, adJointMotion[0].getTPose(), adMcfg)
    wdModel = cvm.VpControlModel(vpWorld, wdJointMotion[0].getTPose(), wdMcfg)

    ankle_l_idx = adModel.name2index("ankle_l")
    ankle_r_idx = adModel.name2index("ankle_r")
    knee_l_idx = adModel.name2index("knee_l")
    hip_l_idx = adModel.name2index("hip_l")
    hip_r_idx = adModel.name2index("hip_r")
    LeftFoot_idx = wdModel.name2index("LeftFoot")
    LeftLeg_idx = wdModel.name2index("LeftLeg")
    LeftUpLeg_idx = wdModel.name2index("LeftUpLeg")
    RightUpLeg_idx = wdModel.name2index("RightUpLeg")

#   rotate wd2 model's Foot joints
    local_zero = wdModel.getBodyPositionGlobal(LeftFoot_idx, (0,0,0))
    local_x_axis = wdModel.getBodyPositionGlobal(LeftFoot_idx, (1,0,0))-local_zero
    local_y_axis = wdModel.getBodyPositionGlobal(LeftFoot_idx, (0,1,0))-local_zero
    local_z_axis = wdModel.getBodyPositionGlobal(LeftFoot_idx, (0,0,1))-local_zero
    local_Transform = np.transpose(np.array([local_x_axis/np.linalg.norm(local_x_axis), local_y_axis/np.linalg.norm(local_y_axis), local_z_axis/np.linalg.norm(local_z_axis)], float))
#    print local_Transform
    wdTPose = wdJointMotion[0].getTPose()
    wdTPose.mulGlobalR(wdTPose.skeleton.getElementIndex("LeftFoot"), local_Transform.T)
    wdTPose.mulGlobalR(wdTPose.skeleton.getElementIndex("RightFoot"), local_Transform.T)
    
#   remake wd2 model with rotated TPose
    wdModel2 = cvm.VpControlModel(vpWorld, wdTPose, wdMcfg)
    
    # wd2
    leftFootNode = wdMcfg.getNode("LeftFoot")
    wd2_footHeight = math.fabs(leftFootNode.mass / (leftFootNode.density * leftFootNode.length * leftFootNode.width * 2.0))
    wd2_ankleHeight = wdModel2.getJointPositionGlobal(LeftFoot_idx)[1] - wdModel2.getBodyPositionGlobal(LeftFoot_idx, (0,-wd2_footHeight,0))[1]
    wd2_uplegVec = wdModel2.getJointPositionGlobal(LeftUpLeg_idx) - wdModel2.getJointPositionGlobal(LeftLeg_idx)
    wd2_legVec = wdModel2.getJointPositionGlobal(LeftLeg_idx) - wdModel2.getJointPositionGlobal(LeftFoot_idx)
    
    wd2_legLength = np.linalg.norm(wd2_uplegVec) + np.linalg.norm(wd2_legVec)
    wd2_upperLeg = (wdModel2.getJointPositionGlobal(LeftUpLeg_idx) + wdModel2.getJointPositionGlobal(RightUpLeg_idx))/2.0
    wd2_upperLegFromRoot = wd2_upperLeg - wdModel2.getJointPositionGlobal(0)

    # ad
    ankle_l_node = adMcfg.getNode("ankle_l")
    ad_footHeight = math.fabs(ankle_l_node.mass / (ankle_l_node.density * ankle_l_node.length * ankle_l_node.width * 2.0))
    ad_ankleHeight = adModel.getJointPositionGlobal(ankle_l_idx)[1] - adModel.getBodyPositionGlobal(ankle_l_idx, (0,-ad_footHeight,0))[1]
    ad_uplegVec = adModel.getJointPositionGlobal(hip_l_idx) - adModel.getJointPositionGlobal(knee_l_idx)
    ad_legVec = adModel.getJointPositionGlobal(knee_l_idx) - adModel.getJointPositionGlobal(ankle_l_idx)
    ad_legLength = np.linalg.norm(ad_uplegVec) + np.linalg.norm(ad_legVec)
    ad_upperLeg = (adModel.getJointPositionGlobal(hip_l_idx) + adModel.getJointPositionGlobal(hip_r_idx))/2.0
    ad_upperLegFromRoot = ad_upperLeg - adModel.getJointPositionGlobal(0)

    return wd2_ankleHeight, wd2_legLength, ad_ankleHeight, ad_legLength

    """
    vpWorld.initialize()
    viewer = ysv.SimpleViewer()
    viewer.doc.addRenderer('ad', yr.JointMotionRenderer(adJointMotion,(0,0,255), yr.POLYGON_LINE))
    viewer.doc.addRenderer('wd', yr.JointMotionRenderer(wdJointMotion,(0,255,0), yr.POLYGON_LINE))
    viewer.doc.addRenderer('adVpModel', cvr.VpModelRenderer(adModel, (200,200,200), yr.POLYGON_LINE))
    viewer.doc.addRenderer('wdVpModel', cvr.VpModelRenderer(wdModel2, (0,255,0), yr.POLYGON_LINE))
    viewer.startTimer(1/30.)
    viewer.show()

    Fl.run()
    """


if __name__=='__main__':
    retargetMotion()
