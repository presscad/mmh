import psyco; psyco.full()
from fltk import *

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm
import Resource.ysMotionLoader as yf
import Motion.ysMotionAnalysis as yma
import Motion.ysBipedAnalysis as yba
import Motion.ysTrajectoryEdit as yte
import GUI.ysSimpleViewer as ysv
import Renderer.ysRenderer as yr
import Simulator.csVpWorld as cvw
import Simulator.csVpModel as cvm
import Motion.ysSkeletonEdit as yme
import Simulator.ysPhysConfig as ypc
import Renderer.csVpRenderer as cvr
import ArticulatedBody.ysReferencePoints as yrp
import Util.ysMatplotEx as ymp


def get_mcfg():
    massMap = {}
    massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips', 'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector', 'LeftLeg', 'LeftShoulder1', 'LeftToes', 'LeftToes_Effector', 'LeftUpLeg', 'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector', 'RightLeg', 'RightShoulder', 'RightToes', 'RightToes_Effector', 'RightUpLeg', 'Spine', 'Spine1'], 0.)
    
    # torso : 10
    massMap['Hips'] += 2.
    massMap['Spine'] += 8.
    
    # head : 3
    massMap['Spine1'] += 3.
    
    # right upper arm : 2
    massMap['RightArm'] += 2.
    
    # left upper arm : 2
    massMap['LeftArm'] += 2.
    
    # right lower arm : 1
#    massMap['RightForeArm'] = 1.
    massMap['RightForeArm'] = 2.
    
    # left lower arm : 1
#    massMap['LeftForeArm'] = 1.
    massMap['LeftForeArm'] = 2.
    
    # right thigh : 7
    massMap['Hips'] += 2.
    massMap['RightUpLeg'] += 5.
    
    # left thigh : 7
    massMap['Hips'] += 2.
    massMap['LeftUpLeg'] += 5.
    
    # right shin : 5
    massMap['RightLeg'] += 5.
    
    # left shin : 5
    massMap['LeftLeg'] += 5.
    
    # right foot : 4
    massMap['RightFoot'] += 2.
    
    # left foot : 4
    massMap['LeftFoot'] += 2.
        

    
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .9

    for name in massMap:
        node = mcfg.addNode(name)
        node.mass = massMap[name]
        
    node = mcfg.getNode('Hips')
    node.length = .2
    node.width = .25
    
    node = mcfg.getNode('Spine1')
    node.length = .2
    node.offset = (0,0,0.1)
    
    node = mcfg.getNode('Spine')
    node.width = .22
    
    node = mcfg.getNode('RightFoot')
    node.length = .25
    
    node = mcfg.getNode('LeftFoot')
    node.length = .25
    
    return mcfg
    
    
g_motionDirConfigMap = {}
g_motionDirConfigMap['../../../Data/woody2/Motion/Physics2/'] = \
    {'footRot': mm.exp(mm.v3(1,0,0), -.4), 'yOffset': .0, 'scale':1.,'rootRot': mm.I_SO3()}
g_motionDirConfigMap['../../../Data/woody2/Motion/Balancing/'] = \
    {'footRot': mm.exp(mm.v3(1,-.5,0), -.6), 'yOffset': .0, 'scale':1.,'rootRot': mm.exp(mm.v3(1,0,0), .01)}
g_motionDirConfigMap['../../../Data/woody2/Motion/Picking/'] = \
    {'footRot': mm.exp(mm.v3(1,0,0), -.5), 'yOffset': .0, 'scale':1.,'rootRot': mm.I_SO3()}
g_motionDirConfigMap['../../../Data/woody2/Motion/Samsung/'] = \
    {'footRot': mm.rotX(-.5), 'yOffset': .0, 'scale':2.53999905501,'rootRot': mm.I_SO3()}
g_motionDirConfigMap['../../../Data/woody2/Motion/VideoMotion/'] = \
    {'footRot': mm.exp(mm.v3(1,0,0), -.05), 'yOffset': .01, 'scale':2.53999905501,'rootRot': mm.exp(mm.v3(1,0,0), .0)}


if __name__=='__main__':
    #===============================================================================
    # initialize motion
    #===============================================================================
    motionFiles = [] 
    
    motionDir = '../../../Data/woody2/Motion/Physics2/'
    motionFiles.append([motionDir, 'wd2_WalkSameSame01.bvh'])
#    motionFiles.append([motionDir, 'wd2_WalkForwardSlow01.bvh'])
#    motionFiles.append([motionDir, 'wd2_WalkForwardNormal00.bvh'])
#    motionFiles.append([motionDir, 'wd2_WalkHandWav00.bvh'])
#    motionFiles.append([motionDir, 'wd2_WalkForwardFast00.bvh'])
#    motionFiles.append([motionDir, 'wd2_WalkForwardVFast00.bvh'])
#    motionFiles.append([motionDir, 'wd2_WalkLean00.bvh'])
#    motionFiles.append([motionDir, 'wd2_WalkAzuma01.bvh'])
#    motionFiles.append([motionDir, 'wd2_WalkSoldier00.bvh'])
#    motionFiles.append([motionDir, 'wd2_WalkSukiko00.bvh'])
    hRef = .1; vRef = .2
    jumpThreshold = 10; jumpBias = 1.
    stopThreshold = 12; stopBias = 0.
        
#    motionDir = '../../../Data/woody2/Motion/Samsung/'
#    motionFiles.append([motionDir, 'wd2_left_turn.bvh'])
#    motionFiles.append([motionDir, 'wd2_right_turn.bvh'])
#    hRef = .15; vRef = .2
#    jumpThreshold = 10; jumpBias = 1.
#    stopThreshold = 12; stopBias = 0.

#    motionDir = '../../../Data/woody2/Motion/VideoMotion/'
#    motionFiles.append([motionDir, 'wd2_spiral_walk01.bvh'])
#    motionFiles.append([motionDir, 'wd2_spiral_walk_fast10.bvh'])
#    motionFiles.append([motionDir, 'wd2_spiral_walk_normal05.bvh'])
#    motionFiles.append([motionDir, 'wd2_spiral_walk_slow02.bvh'])
#    hRef = .15; vRef = .2
#    jumpThreshold = 10; jumpBias = 1.
#    stopThreshold = 12; stopBias = 0.
    
    VISUALIZE = True
    STEP_INFO = True

    for i in range(len(motionFiles)):
        motionDir = motionFiles[i][0]
        motionName = motionFiles[i][1]
        
        bvhFilePath = motionDir + motionName

        motionDirConfig = g_motionDirConfigMap[motionDir]
        motion = yf.readBvhFile(bvhFilePath, .01*motionDirConfig['scale'])
        
        #=======================================================================
        # for motionDir = '../../../Data/woody2/Motion/Physics2/'
        #=======================================================================
        yme.removeJoint(motion, 'Head', False)
        yme.removeJoint(motion, 'RightShoulder', False)
        yme.removeJoint(motion, 'LeftShoulder1', False)
        yme.removeJoint(motion, 'RightToes_Effector', False)
        yme.removeJoint(motion, 'LeftToes_Effector', False)
        yme.removeJoint(motion, 'RightHand_Effector', False)
        yme.removeJoint(motion, 'LeftHand_Effector', False)
        yme.offsetJointLocal(motion, 'RightArm', (.03,-.05,0), False)
        yme.offsetJointLocal(motion, 'LeftArm', (-.03,-.05,0), False)

        #=======================================================================
        # for motionDir = '../../../Data/woody2/Motion/Samsung/'
        #=======================================================================
#        yme.removeJoint(motion, 'RightUpLegDummy', False)
#        yme.removeJoint(motion, 'SpineDummy', False)
#        yme.removeJoint(motion, 'HEadDummy', False)
#        yme.removeJoint(motion, 'LeftShoulder1Dummy', False)
#        yme.removeJoint(motion, 'RightShoulderDummy', False)
#        yme.removeJoint(motion, 'LeftUpLegDummy', False)
#        yme.removeJoint(motion, 'Head', False)
#        yme.removeJoint(motion, 'RightShoulder', False)
#        yme.removeJoint(motion, 'LeftShoulder1', False)
#        yme.removeJoint(motion, 'RightToes_Effector', False)
#        yme.removeJoint(motion, 'LeftToes_Effector', False)
#        yme.removeJoint(motion, 'RightHand_Effector', False)
#        yme.removeJoint(motion, 'LeftHand_Effector', False)
#        yme.offsetJointLocal(motion, 'RightArm', (.03,-.05,0), False)
#        yme.offsetJointLocal(motion, 'LeftArm', (-.03,-.05,0), False)

        #=======================================================================
        # common
        #=======================================================================
        yme.rotateJointLocal(motion, 'Hips', motionDirConfig['rootRot'], False)
        yme.rotateJointLocal(motion, 'LeftFoot', motionDirConfig['footRot'], False)
        yme.rotateJointLocal(motion, 'RightFoot', motionDirConfig['footRot'], False)
        motion.translateByOffset(motionDirConfig['yOffset'])
        yme.updateGlobalT(motion)

        print motionName
        
        if motionName=='wd2_left_turn.bvh':
            lFoot = motion[0].skeleton.getJointIndex('LeftFoot')
            yte.setPositionTarget(motion, lFoot, motion[0].getJointPositionGlobal(lFoot)+(-.1,0,-.1)\
                                  , [0,58], 100)

        #===============================================================================
        # initialize character
        #===============================================================================
        wcfg = ypc.WorldConfig()
        mcfg = get_mcfg()
        vpWorld = cvw.VpWorld(wcfg)
        motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
        vpWorld.initialize()
        
        bodyMasses = motionModel.getBodyMasses()
        totalMass = motionModel.getTotalMass()
        
        #===============================================================================
        # analysis
        #===============================================================================
        lc = yma.getElementContactStates(motion, 'LeftFoot', hRef, vRef)
        rc = yma.getElementContactStates(motion, 'RightFoot', hRef, vRef)
        
        intervals, states = yba.getBipedGaitIntervals(lc, rc, jumpThreshold, jumpBias, stopThreshold, stopBias)
        segments = yma.splitMotionIntoSegments(motion, intervals)
    
        lFoot = motion[0].skeleton.getJointIndex('LeftFoot')
        rFoot = motion[0].skeleton.getJointIndex('RightFoot')
    
        CM_vels = [None]*len(intervals)
        stepLengths = [None]*len(intervals)
        stepDurations = [None]*len(intervals)
    
        print '                              CM vel    step length    step duration'
        for i in range(len(intervals)):
            startFrame = intervals[i][0]
            endFrame = intervals[i][1]
            
            stepDurations[i] = (endFrame - startFrame) * 1/30.
    
            motionModel.update(motion[startFrame])
            CM0 = yrp.getCM(motionModel.getBodyPositionsGlobal(), bodyMasses, totalMass)
            motionModel.update(motion[endFrame])
            CM1 = yrp.getCM(motionModel.getBodyPositionsGlobal(), bodyMasses, totalMass)
            CM_vels[i] = mm.length((CM1 - CM0) / stepDurations[i])
    
            lSwingLength = mm.length(motion[endFrame].getJointPositionGlobal(lFoot) - motion[startFrame].getJointPositionGlobal(lFoot))
            rSwingLength = mm.length(motion[endFrame].getJointPositionGlobal(rFoot) - motion[startFrame].getJointPositionGlobal(rFoot))
            stepLengths[i] = max([lSwingLength, rSwingLength])
            
            if STEP_INFO:
                print '%2dth'%i, '%-6s'%yba.GaitState.text[states[i]], '%-10s'%intervals[i], '%2d'%(endFrame-startFrame), '%10.2f %10.2f %10.2f'%(CM_vels[i], stepLengths[i], stepDurations[i])
    
        startSeg = 1
        endSeg = len(intervals)-2
        mean_CM_vel = sum(CM_vels[startSeg:endSeg+1]) / (endSeg - startSeg + 1)
        mean_stepLength = sum(stepLengths[startSeg:endSeg+1]) / (endSeg - startSeg + 1)
        mean_stepDuration = sum(stepDurations[startSeg:endSeg+1]) / (endSeg - startSeg + 1)
        print 'mean (%dth~%dth)         %10.2f %10.2f %10.2f'%(startSeg, endSeg, mean_CM_vel, mean_stepLength, mean_stepDuration)
        print
    
    
        if VISUALIZE:
            #===============================================================================
            # plotting
            #===============================================================================
            rawStates = yba.getBipedGaitStates(lc, rc)
            refinedStates = yba.getBipedGaitStates(lc, rc, jumpThreshold, jumpBias, stopThreshold, stopBias)
            
            plot = ymp.SmartPlot()
            plot.setXdata('frame', range(len(motion)))
            plot.addYdata('rawStates', rawStates)
            plot.addYdata('refinedStates', refinedStates)
            plot.showModeless()
            
            #===============================================================================
            # viewer
            #===============================================================================
            viewer = ysv.SimpleViewer()
            viewer.record(False)
            viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,100,255), yr.LINK_LINE))
            viewer.doc.addObject('motion', motion)
    #        viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, (0,100,255), yr.POLYGON_LINE))
            
            def postFrameCallback_Always(frame):
                motionModel.update(motion[frame])
            viewer.setPostFrameCallback_Always(postFrameCallback_Always) 
            
            viewer.startTimer(1/30.)
            viewer.show()
            Fl.run()