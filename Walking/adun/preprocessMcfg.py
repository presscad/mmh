import cPickle

import sys
if '../../PyCommon/modules' not in sys.path:
        sys.path.append('../../PyCommon/modules')
import Simulator.ysPhysConfig as ypc
import Resource.dcVRMLLoader as dvl
import numpy.core.multiarray
import renameJoint as rj
import Math.mmMath as mm
import math

def buildMassMapFromWrl(wrl):
    massMap = {}
    centerOfMassMap = {}
    momentsOfInertiaMap = {}

    for joint in wrl.joints:
        childSegment = joint.getChildSegment()
        massMap[joint.name] = childSegment.mass
        centerOfMassMap[joint.name] = childSegment.centerOfMass
        momentsOfInertiaMap[joint.name] = childSegment.momentsOfInertia

    return massMap, centerOfMassMap, momentsOfInertiaMap

def applyCenterOfMassMap(mcfg, centerOfMassMap):
    for node in mcfg.nodes.values():
        node.localT = centerOfMassMap[node.name]

    return mcfg

def clearLocalRAt(mcfg, nodeNames):
    for name in nodeNames:
        node = mcfg.getNode(name)
        node.localR = mm.I_SO3()

def adjustFoot(mcfg, wrl):
    feet = [["ankle_l", "mtp_l"], ["ankle_r", "mtp_r"]]
    for foot in feet:
        ankleNode = mcfg.getNode(foot[0])
        mtpNode = mcfg.getNode(foot[1])
        ankleJoint = wrl.getWrlJoint(foot[0])
        mtpJoint = wrl.getWrlJoint(foot[1])
        
        defaultHeight = 0.03
        mtpHeight = math.fabs(mtpJoint.getChildSegment().centerOfMass[1] * 2.) + defaultHeight
        ankleHeight = math.fabs(mtpJoint.translation[1] + (mtpJoint.getChildSegment().centerOfMass[1] * 2.) - ankleJoint.getChildSegment().centerOfMass[1]) + defaultHeight
        mtpNode.width = 0.06
        ankleNode.width = 0.20
        mtpNode.length = (mtpNode.mass / mtpNode.density) / (mtpHeight * mtpNode.width)
        ankleNode.length = (ankleNode.mass / ankleNode.density) / (ankleHeight * ankleNode.width)

    return mcfg

def adjustUpperBody(mcfg, wrl):
    
    pelvisNode = mcfg.getNode("ground_pelvis")
    backNode = mcfg.getNode("back")

    pelvisNode.length = 0.3
    pelvisNode.width = 0.15
    pelvisNode.offset = (-0.07,0.1,0)

    backNode.localT[1] = backNode.localT[1] - 0.30
    backNode.length = 0.5
    backNode.width = 0.13
    backNode.density = 2000
    backNode.offset = (0.03,0,0)

    elbowLNode = mcfg.getNode("elbow_l")
    elbowRNode = mcfg.getNode("elbow_r")
    
    elbowLNode.localT = None
    elbowLNode.width = 0.05
    elbowLNode.density = 16000

    elbowRNode.localT = None
    elbowRNode.width = 0.05
    elbowRNode.density = 16000

    radLNode = mcfg.getNode("radioulnar_l")
    radRNode = mcfg.getNode("radioulnar_r")
    
    return mcfg

def configureMcfg(outputFilePath, wrl):

    massMap, centerOfMassMap, momentsOfInertiaMap = buildMassMapFromWrl(wrl)

    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.0
    mcfg.defaultBoneRatio = .9

    totalMass = 0.
    for name in massMap:
        node = mcfg.addNode(name)
        node.mass = massMap[name]
        totalMass += node.mass
    print totalMass

    if centerOfMassMap != None:
        applyCenterOfMassMap(mcfg, centerOfMassMap)

    clearLocalRAt(mcfg, ["ground_pelvis", "ankle_l", "mtp_l", "ankle_r", "mtp_r"])

    adjustFoot(mcfg, wrl)

    adjustUpperBody(mcfg, wrl)

    outputFile = open(outputFilePath, 'wb')
    cPickle.dump(mcfg, outputFile)
    outputFile.close()

    print outputFilePath, 'done'
    print 'FINISHED'

def reconfigureMcfg(mcfgFilePath):

    mcfgFile = open(mcfgFilePath, 'rb')
    mcfg = cPickle.load(mcfgFile)
    mcfgFile.close()

    for node in mcfg.nodes.values():
        print node.name, node.mass
#        node.density = node.mass / 0.001

    #node.length : local z axis direction
    #node.width : local x axis direction

#    node = mcfg.getNode("hip_l")
#    node.offset = (1,0,0)
#    node.offset = (0,1,0)
#    node.offset = (0,0,1)

#    node = mcfg.getNode("knee_l")
#    node.offset = (1,0,0)
#    node.offset = (0,1,0)
#    node.offset = (0,0,1)

#    node = mcfg.getNode("ankle_l")
#    node.offset = (1,0,0)
#    node.offset = (0,1,0)
#    node.offset = (0,0,1)

    mcfgFile = open(mcfgFilePath, 'wb')
    cPickle.dump(mcfg, mcfgFile)
    mcfgFile.close()

if __name__=='__main__':

    mcfgFilePath = './mcfg'
    reconfigureMcfg(mcfgFilePath)

    """
    wrlFilePath = './osim/FullBody2_lee.wrl'
    wrl = dvl.readWrlFileasWrl(wrlFilePath)

    outputDir = './'
    outputName = 'mcfg'
    
    configureMcfg(outputDir+outputName, wrl)
    """
