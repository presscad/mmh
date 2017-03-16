import cPickle

import sys
if '../../../PyCommon/modules' not in sys.path:
        sys.path.append('../../../PyCommon/modules')
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
        mtpDepth = 0.02
        ankleDepth = (math.fabs(mtpJoint.translation[1] + mtpJoint.getChildSegment().centerOfMass[1]) - math.fabs(ankleJoint.getChildSegment().centerOfMass[1]))*2.0 + mtpDepth
        mtpNode.length = 0.09
        mtpNode.density *= 2
        ankleNode.density *= 0.8
        ankleNode.length = 0.12
        mtpNode.width = (mtpNode.mass / mtpNode.density) / (mtpDepth * mtpNode.length)
        ankleNode.width = (ankleNode.mass / ankleNode.density) / (ankleDepth * ankleNode.length)

    return mcfg

def adjustUpperBody(mcfg, wrl):
    
    pelvisNode = mcfg.getNode("ground_pelvis")
    backNode = mcfg.getNode("back")

    pelvisNode.length = 0.3
    pelvisNode.width = 0.15
    pelvisNode.offset = (-0.007,0.1,0)

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

def modifyMass(mcfg):

    nodes = ['back', 'shoulder_l', 'elbow_l', 'radioulnar_l', 'radius_hand_l', 'shoulder_r', 'elbow_r', 'radioulnar_r', 'radius_hand_r']
    for nodeName in nodes:
        node = mcfg.getNode(nodeName)
        node.mass = node.mass / 10.

    totalMass = 0.
    for nodeName in mcfg.nodes:
        totalMass += mcfg.nodes[nodeName].mass
    print totalMass

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
    
#    modifyMass(mcfg)

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

    #mcfgFilePath = './mcfg'
    #reconfigureMcfg(mcfgFilePath)

    # FullBody2_lee.wrl needs to rename joints
    # Use preprocessWrl.py
    '''    
    wrlFilePath = './osim/FullBody2_lee.wrl'
    wrl = dvl.readWrlFileasWrl(wrlFilePath)

    outputDir = './'
    outputName = 'mcfg'
    
    configureMcfg(outputDir+outputName, wrl)
    '''
