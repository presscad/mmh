import psyco; psyco.full()
from fltk import *
import copy, time, math, cPickle
import numpy as np

import sys
sys.path.append('..')
import GUI.ysSimpleViewer as ysv
import Resource.ysOgreDataLoader as yol
import Resource.ysMotionLoader as yf
import Renderer.ysRenderer as yr
import Util.ysPythonEx as ype
import Motion.ysSkeletonEdit as yme
import Math.mmMath as mm
import Simulator.csVpModel as cvm
import Simulator.csVpWorld as cvw
import Simulator.ysPhysConfig as ypc
import Renderer.csVpRenderer as cvr
import ArticulatedBody.ysReferencePoints as yrp
import ArticulatedBody.ysControl as yct


def test_VpMotionModel():
    bvhFilePath = '../samples/block_3_rotate.bvh'
    motion = yf.readBvhFile(bvhFilePath)
    
    wcfg = ypc.WorldConfig()
    vpWorld = cvw.VpWorld(wcfg)
    
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .8
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
    node = mcfg.getNode('body1')
    node.mass = 1.
    node = mcfg.getNode('body2')
    node.mass = 1.
    node.boneRatio = .5
    node.offset = (0,0,.1)
    node = mcfg.getNode('body3')
    node.mass = 1.
    node.length = .1
#    mcfg.delNode('body2')

    motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
    
    vpWorld.initialize()
    
    viewer = ysv.SimpleViewer()
    viewer.record(False)
    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, (255,240,255), cvr.POLYGON_LINE))
    
    def preFrameCallback(frame):
        motionModel.update(motion[frame])
    viewer.setPreFrameCallback(preFrameCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run() 
    
def test_VpControlModel():
    bvhFilePath = '../samples/block_3_rotate.bvh'
#    bvhFilePath = '../samples/block_tree_rotate.bvh'
    motion = yf.readBvhFile(bvhFilePath)
    motion = motion[30:]
    
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .8
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
    node = mcfg.getNode('body2')
    node.length = 1
    node.offset = (0,0,.2)
#    mcfg.delNode('body2')
    
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = -1.
    stepsPerFrame = 30
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    
    viewer = ysv.SimpleViewer()
#    viewer.record(False)
    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('model', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_FILL))

    def simulateCallback(frame):
        for i in range(stepsPerFrame):
            vpWorld.step()
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()     
                     
def buildMassMap():
    massMap = {}
    massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips', 'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector', 'LeftLeg', 'LeftShoulder1', 'LeftToes', 'LeftToes_Effector', 'LeftUpLeg', 'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector', 'RightLeg', 'RightShoulder', 'RightToes', 'RightToes_Effector', 'RightUpLeg', 'Spine', 'Spine1'], 0.)
    
    # torso : 10
    massMap['Hips'] += 2.
    massMap['Spine'] += 6.
    massMap['Spine1'] += 2.
    
    # head : 3
    massMap['Spine1'] += 1.
    massMap['Head'] += 2.
    
    # right upper arm : 2
    massMap['Spine1'] += .5
    massMap['RightShoulder'] += .5
    massMap['RightArm'] += 1.
    
    # left upper arm : 2
    massMap['Spine1'] += .5
    massMap['LeftShoulder1'] += .5
    massMap['LeftArm'] += 1.
    
    # right lower arm : 1
    massMap['RightForeArm'] = .8
    massMap['RightHand'] = .2 
    
    # left lower arm : 1
    massMap['LeftForeArm'] = .8
    massMap['LeftHand'] = .2 
    
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
    massMap['RightToes'] += 2.
    
    # left foot : 4
    massMap['LeftFoot'] += 2.
    massMap['LeftToes'] += 2.
    
    return massMap

def test_biped_delNode():
    massMap = buildMassMap()
    
    bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
    motion = yf.readBvhFile(bvhFilePath, .01)
    
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .9

    for name in massMap:
        node = mcfg.addNode(name)
        node.mass = massMap[name]
        
    node = mcfg.getNode('Hips')
    node.length = .2
    node.width = .25
    
    node = mcfg.getNode('Head')
    node.length = .2
    
    node = mcfg.getNode('Spine')
    node.width = .22
    
    node = mcfg.getNode('RightFoot')
    node.length = .25
    node = mcfg.getNode('LeftFoot')
    node.length = .25
    
    mcfg.delNode('Spine1')
    mcfg.delNode('RightShoulder')
    mcfg.delNode('LeftShoulder1')
    mcfg.delNode('RightHand')
    mcfg.delNode('LeftHand')
    mcfg.delNode('RightToes')
    mcfg.delNode('LeftToes')
    
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    stepsPerFrame = 30
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    
    motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
    
    vpWorld.initialize()
    
    viewer = ysv.SimpleViewer()
#    viewer.record(False)
    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_LINE))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, (255,240,255), cvr.POLYGON_FILL))
    viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, (255,240,255), cvr.POLYGON_LINE))
    
    def preFrameCallback(frame):
        motionModel.update(motion[frame])
    viewer.setPreFrameCallback(preFrameCallback)
    
    def simulateCallback(frame):
        for i in range(stepsPerFrame):
            vpWorld.step()
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()         
    
def test_biped_motion_edit():
    massMap = buildMassMap()
    
    bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
    motion = yf.readBvhFile(bvhFilePath, .01)
    yme.removeJoint(motion, 'Head', False)
    yme.removeJoint(motion, 'RightShoulder', False)
    yme.removeJoint(motion, 'LeftShoulder1', False)
    yme.removeJoint(motion, 'RightToes_Effector', False)
    yme.removeJoint(motion, 'LeftToes_Effector', False)
    yme.removeJoint(motion, 'RightHand_Effector', False)
    yme.removeJoint(motion, 'LeftHand_Effector', False)
    yme.offsetJointLocal(motion, 'RightArm', (.03,-.05,0), False)
    yme.offsetJointLocal(motion, 'LeftArm', (-.03,-.05,0), False)
    yme.updateGlobalT(motion)
    
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .9

    for name in massMap:
        node = mcfg.addNode(name)
        node.mass = massMap[name]
        
    node = mcfg.getNode('Hips')
    node.length = .2
    node.width = .25
    
#    node = mcfg.getNode('Head')
#    node.length = .2
    
    node = mcfg.getNode('Spine1')
    node.length = .2
    node.offset = (0,0,0.1)

    node = mcfg.getNode('Spine')
    node.width = .22

    node = mcfg.getNode('RightFoot')
    node.length = .25
    node = mcfg.getNode('LeftFoot')
    node.length = .25
    
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    stepsPerFrame = 30
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    
    print motion[0].skeleton
    print
    print controlModel
    
    viewer = ysv.SimpleViewer()
#    viewer.record(False)
    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,255,255), yr.LINK_LINE))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, (255,240,255), cvr.POLYGON_FILL))
    viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, (255,240,255), cvr.POLYGON_LINE))
    
    def preFrameCallback(frame):
        motionModel.update(motion[frame])
    viewer.setPreFrameCallback(preFrameCallback)
    
    def simulateCallback(frame):
        for i in range(stepsPerFrame):
            vpWorld.step()
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()         
                

def test_hybridDynamics():
    bvhFilePath = '../samples/chain_2.bvh'
    motion = yf.readBvhFile(bvhFilePath)
    
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .8
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
    
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = -1.
    wcfg.gravity = (0,0,0)
    stepsPerFrame = 30
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    controlModel2 = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    
    controlModel.fixBody(0)
    controlModel.initializeHybridDynamics()
    controlModel2.fixBody(0)
    controlModel2.initializeHybridDynamics()
    
    controlModel2.translateByOffset((0,1,0))
    
    p = []
    torques = []
    
    viewer = ysv.SimpleViewer()
    viewer.record(False)
    viewer.doc.addRenderer('model', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('model2', cvr.VpModelRenderer(controlModel2, (255,240,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('torques', yr.VectorsRenderer(torques, p, (255,0,0)))
    viewer.setMaxFrame(100)

    def simulateCallback(frame):
        p[:] = controlModel.getInternalJointPositionsGlobal() + controlModel2.getInternalJointPositionsGlobal()
        
        for i in range(stepsPerFrame):
            controlModel.setJointAngAccelerationLocal(1, (0,0,1))
            controlModel.solveHybridDynamics()
            
            controlModel2.setJointAngAccelerationLocal(1, (0,0,1))
            controlModel2.applyBodyForceGlobal(1, (0,1,0))
            controlModel2.solveHybridDynamics()
            
            torques[:] = controlModel.getInternalJointTorquesLocal() + controlModel2.getInternalJointTorquesLocal() 
            
            vpWorld.step()
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()     
                     
def test_hybridDynamics_gravity():
    bvhFilePath = '../samples/chain_1.bvh'
    motion = yf.readBvhFile(bvhFilePath)
    
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .8
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
    
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = -1.
#    wcfg.gravity = (0,0,0)
    stepsPerFrame = 30
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    controlModel2 = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    
    controlModel.initializeHybridDynamics()
    controlModel2.initializeHybridDynamics()
    
    controlModel2.translateByOffset((1,0,0))
    
    p = []
    torques = []
    
    viewer = ysv.SimpleViewer()
    viewer.record(False)
    viewer.doc.addRenderer('model', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('model2', cvr.VpModelRenderer(controlModel2, (255,240,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('torques', yr.VectorsRenderer(torques, p, (255,0,0)))
    viewer.setMaxFrame(100)

    def simulateCallback(frame):
        p[:] = controlModel.getInternalJointPositionsGlobal() + controlModel2.getInternalJointPositionsGlobal()
        
        for i in range(stepsPerFrame):
            controlModel.solveHybridDynamics()
            
            print 'a', controlModel2.getBodyForceLocal(0)
            controlModel2.applyBodyForceGlobal(0, (0,10,0))
            print 'b', controlModel2.getBodyForceLocal(0)
            controlModel2.solveHybridDynamics()
            
            torques[:] = controlModel.getInternalJointTorquesLocal() + controlModel2.getInternalJointTorquesLocal() 
            
            vpWorld.step()
            print 'c', controlModel2.getBodyForceLocal(0)
            
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()     
                     
def test_joint_pos_vel_acc_funcs_and_tracking():
    def getDesiredAngAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt):
        ddth_des = [None]*len(th_r) 
        for i in range(len(th_r)):
            ddth_des[i] = Kt*(mm.logSO3(np.dot(th[i].transpose(), th_r[i]))) + Dt*(dth_r[i] - dth[i]) + ddth_r[i]
        return ddth_des
    def getDesiredAcceleration(p_r, p, v_r, v, a_r, Kt, Dt):
        return Kt*(p_r - p) + Dt*(v_r - v) + a_r
    
#    bvhFilePath = '../samples/chain_1.bvh'
#    bvhFilePath = '../samples/block_tree_rotate.bvh'
    bvhFilePath = '../samples/chain_3_rotate_freely_move.bvh'
#    bvhFilePath = '../samples/chain_3_rotate_freely.bvh'
#    bvhFilePath = '../samples/chain_3_rotate_freely_expt_root.bvh'
#    bvhFilePath = '../samples/chain_3_rotate.bvh'
#    bvhFilePath = '../samples/chain_3_rotate_expt_root.bvh'
#    bvhFilePath = '../samples/chain_6_rotate_expt_root.bvh'
#    bvhFilePath = '../samples/chain_2_rotate_2axes.bvh'
    motion = yf.readBvhFile(bvhFilePath)
    
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .8
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
    
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = -1.
    wcfg.gravity = (0,0,0)
    stepsPerFrame = 30
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    controlModel.initializeHybridDynamics(False)
    
#    controlModel.fixBody(0)
    controlModel.rotate(mm.exp(mm.v3(0,1,0)))
    print controlModel
    
    motion_p = []; motion_v = []; motion_a = []
    motion_ap = []; motion_av = []; motion_aa = []
    motion_ap_local = []; motion_av_local = []; motion_aa_local = []

    model_p = []; model_v = []; model_a = []
    model_ap = []; model_av = []; model_aa = []
    model_ap_local = []; model_av_local = []; model_aa_local = []
        
    model_body_p = []; model_body_a = []
    
    prev_model_v = [(0.,0.,0.)]*controlModel.getJointNum()
    prev_model_av = [(0.,0.,0.)]*controlModel.getJointNum()
    prev_model_av_local = [(0.,0.,0.)]*controlModel.getJointNum()

    viewer = ysv.SimpleViewer()
    viewer.record(False)
    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('model', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_LINE))
    
#    viewer.doc.addRenderer('motion_p', yr.PointsRenderer(motion_p, (255,0,0)))
#    viewer.doc.addRenderer('model_p', yr.PointsRenderer(model_p, (0,255,0)))
#
#    viewer.doc.addRenderer('motion_v', yr.VectorsRenderer(motion_v, motion_p, (255,0,0)))
#    viewer.doc.addRenderer('model_v', yr.VectorsRenderer(model_v, model_p, (0,255,0)))

    viewer.doc.addRenderer('motion_a', yr.VectorsRenderer(motion_a, motion_p, (255,0,0)))
    viewer.doc.addRenderer('model_a', yr.VectorsRenderer(model_a, model_p, (0,255,0)))
#    viewer.doc.addRenderer('model_body_a', yr.VectorsRenderer(model_body_a, model_body_p, (255,255,0)))

#    viewer.doc.addRenderer('motion_ap', yr.OrientationsRenderer(motion_ap, motion_p, (255,0,0)))
#    viewer.doc.addRenderer('model_ap', yr.OrientationsRenderer(model_ap, model_p, (0,255,0)))
#    viewer.doc.addRenderer('motion_ap_local', yr.OrientationsRenderer(motion_ap_local, motion_p, (255,100,100)))
#    viewer.doc.addRenderer('model_ap_local', yr.OrientationsRenderer(model_ap_local, model_p, (100,255,100)))

#    viewer.doc.addRenderer('motion_av', yr.VectorsRenderer(motion_av, motion_p, (255,0,0)))
#    viewer.doc.addRenderer('model_av', yr.VectorsRenderer(model_av, model_p, (0,255,0)))
#    viewer.doc.addRenderer('motion_av_local', yr.VectorsRenderer(motion_av_local, motion_p, (255,100,100)))
#    viewer.doc.addRenderer('model_av_local', yr.VectorsRenderer(model_av_local, model_p, (100,255,100)))

#    viewer.doc.addRenderer('motion_aa', yr.VectorsRenderer(motion_aa, motion_p, (255,0,0)))
#    viewer.doc.addRenderer('model_aa', yr.VectorsRenderer(model_aa, model_p, (0,255,0)))
#    viewer.doc.addRenderer('motion_aa_local', yr.VectorsRenderer(motion_aa_local, motion_p, (255,100,100)))
#    viewer.doc.addRenderer('model_aa_local', yr.VectorsRenderer(model_aa_local, model_p, (100,255,100)))
    
    Kt = 200; Dt = 2*(Kt**.5)
    
    def simulateCallback(frame):
        th_r = motion.getJointOrientationsLocal(frame)
        th = controlModel.getJointOrientationsLocal()
        dth_r = motion.getJointAngVelocitiesLocal(frame)
        dth = controlModel.getJointAngVelocitiesLocal()
        ddth_r = motion.getJointAngAccelerationsLocal(frame)
        ddth_des = getDesiredAngAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt)
        
        p_r = motion.getJointPositionGlobal(0, frame)
        p = controlModel.getJointPositionGlobal(0)
        v_r = motion.getJointVelocityGlobal(0, frame)
        v = controlModel.getJointVelocityGlobal(0)
        a_r = motion.getJointAccelerationGlobal(0, frame)
        a_des = getDesiredAcceleration(p_r, p, v_r, v, a_r, Kt, Dt) 
        
        th_r0 = motion.getJointOrientationGlobal(0, frame)
        th0 = controlModel.getJointOrientationGlobal(0)
        dth_r0 = motion.getJointAngVelocityGlobal(0, frame)
        dth0 = controlModel.getJointAngVelocityGlobal(0)
        ddth_r0 = motion.getJointAngAccelerationGlobal(0, frame)
        ddth_des0 = getDesiredAngAccelerations([th_r0], [th0], [dth_r0], [dth0], [ddth_r0], Kt, Dt)[0]
                
        for i in range(stepsPerFrame):
#            controlModel.setBodyAccelerationGlobal(0, a_des)
#            controlModel.setJointAngAccelerationsLocal(ddth_des)

            controlModel.setJointAccelerationGlobal(0, a_des)
            controlModel.setJointAngAccelerationGlobal(0, ddth_des0)
            controlModel.setInternalJointAngAccelerationsLocal(ddth_des[1:])
            
            controlModel.solveHybridDynamics()

            vpWorld.step()
            
        motion_p[:] = motion.getJointPositionsGlobal(frame)
        motion_v[:] = motion.getJointVelocitiesGlobal(frame)
        motion_a[:] = motion.getJointAccelerationsGlobal(frame)
        motion_ap[:] = motion.getJointOrientationsGlobal(frame)
        motion_av[:] = motion.getJointAngVelocitiesGlobal(frame)
        motion_aa[:] = motion.getJointAngAccelerationsGlobal(frame)
        motion_ap_local[:] = motion.getJointOrientationsLocal(frame)
        motion_av_local[:] = motion.getJointAngVelocitiesLocal(frame)
        motion_aa_local[:] = motion.getJointAngAccelerationsLocal(frame)
        
        model_p[:] = controlModel.getJointPositionsGlobal()
        model_v[:] = controlModel.getJointVelocitiesGlobal()
        
#        model_a[:] = controlModel.getJointAccelerationsGlobal()
        model_a[:] = map(lambda v1,v0: (v1-v0)/(1/30.), model_v, prev_model_v)
        prev_model_v[:] = model_v

        model_ap[:] = controlModel.getJointOrientationsGlobal()
        model_av[:] = controlModel.getJointAngVelocitiesGlobal()
        
#        model_aa[:] = controlModel.getJointAngAccelerationsGlobal()
        model_aa[:] = map(lambda v1,v0: (v1-v0)/(1/30.), model_av, prev_model_av)
        prev_model_av[:] = model_av
        
        model_ap_local[:] = controlModel.getJointOrientationsLocal()
        model_av_local[:] = controlModel.getJointAngVelocitiesLocal()
        
#        model_aa_local[:] = controlModel.getJointAngAccelerationsLocal()
        model_aa_local[:] = map(lambda v1,v0: (v1-v0)/(1/30.), model_av_local, prev_model_av_local)
        prev_model_av_local[:] = model_av_local
        
        model_body_p[:] = controlModel.getBodyPositionsGlobal()
        model_body_a[:] = controlModel.getBodyAccelerationsGlobal()
            
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()
                                              
def test_joint_force_funcs():
    bvhFilePath = '../samples/block_tree_rotate.bvh'
    motion = yf.readBvhFile(bvhFilePath)
    
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .8
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
    
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = -1.
    wcfg.gravity = (0,0,0)
    stepsPerFrame = 30
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()

    controlModel.fixBody(0)

    joint_p = []
    joint_t = []

    viewer = ysv.SimpleViewer()
    viewer.record(False)
#    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('model', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_LINE))
    viewer.doc.addRenderer('joint_p', yr.PointsRenderer(joint_p, (0,255,0)))
    viewer.doc.addRenderer('joint_t', yr.VectorsRenderer(joint_t, joint_p, (0,255,0)))

    def simulateCallback(frame):
        controlModel.setJointTorqueLocal(1, (0,0,1))
        
        joint_p[:] = controlModel.getInternalJointPositionsGlobal()
        joint_t[:] = controlModel.getInternalJointTorquesLocal()

        for i in range(stepsPerFrame):
            vpWorld.step()
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()
                                              
def test_body_pos_vel_acc_funcs():
#    bvhFilePath = '../samples/chain_6.bvh'
#    bvhFilePath = '../samples/block_3_rotate.bvh'
    bvhFilePath = '../samples/block_tree_rotate.bvh'
    motion = yf.readBvhFile(bvhFilePath)
    
    bvhFilePath = '../samples/chain_1.bvh'
    motion2 = yf.readBvhFile(bvhFilePath)
    
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .8
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
#    mcfg.getNode('root').density = 1000000.

    mcfg2 = ypc.ModelConfig()
    for i in range(motion2[0].skeleton.getElementNum()):
        mcfg2.addNode(motion2[0].skeleton.getElementName(i))
    
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
#    wcfg.gravity = (0,0,0)
    stepsPerFrame = 30
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    controlModel2 = cvm.VpControlModel(vpWorld, motion2[0], mcfg2)
    vpWorld.initialize()
    
    controlModel.fixBody(0)
    motionModel.recordVelByFiniteDiff()
    controlModel2.setBodyPositionGlobal(0, (0,1,-1))
    
    cm_p = [mm.O_Vec3()]*controlModel.getBodyNum()
    cm_v = [mm.O_Vec3()]*controlModel.getBodyNum()
    cm_a = [mm.O_Vec3()]*controlModel.getBodyNum()
    cm_av = [mm.O_Vec3()]*controlModel.getBodyNum()
    cm_aa = [mm.O_Vec3()]*controlModel.getBodyNum()
    
    mm_p = [mm.O_Vec3()]*motionModel.getBodyNum()
    mm_v = [mm.O_Vec3()]*motionModel.getBodyNum()
    mm_a = [mm.O_Vec3()]*motionModel.getBodyNum()
    mm_av = [mm.O_Vec3()]*motionModel.getBodyNum()
    mm_aa = [mm.O_Vec3()]*motionModel.getBodyNum()
    

    viewer = ysv.SimpleViewer()
    viewer.record(False)
#    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_LINE))
    viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, (100,100,100), yr.POLYGON_LINE))
    viewer.doc.addRenderer('controlModel2', cvr.VpModelRenderer(controlModel2, (255,240,255), yr.POLYGON_LINE))
    
#    viewer.doc.addRenderer('cm_p', yr.PointsRenderer(cm_p, (0,0,255)))
    viewer.doc.addRenderer('cm_v', yr.VectorsRenderer(cm_v, cm_p, (255,0,0)))
#    viewer.doc.addRenderer('cm_a', yr.VectorsRenderer(cm_a, cm_p, (0,255,0)))
#    viewer.doc.addRenderer('cm_av', yr.VectorsRenderer(cm_av, cm_p, (255,255,0)))
#    viewer.doc.addRenderer('cm_aa', yr.VectorsRenderer(cm_aa, cm_p, (0,255,255)))

#    viewer.doc.addRenderer('mm_p', yr.PointsRenderer(mm_p, (200,200,0)))
    viewer.doc.addRenderer('mm_v', yr.VectorsRenderer(mm_v, mm_p, (200,200,0)))
#    viewer.doc.addRenderer('mm_a', yr.VectorsRenderer(mm_a, mm_p, (200,200,0)))
#    viewer.doc.addRenderer('mm_av', yr.VectorsRenderer(mm_av, mm_p, (0,200,0)))

    controlModel.applyBodyTorqueGlobal(1, (0,0,200))
    controlModel2.applyBodyTorqueGlobal(0, (0,0,200))
    
    def simulateCallback(frame):

        for i in range(stepsPerFrame):
            vpWorld.step()
            
        motionModel.update(motion[frame])
#        controlModel.applyBodyTorqueGlobal(1, (0,0,10))
#        controlModel2.applyBodyTorqueGlobal(0, (0,0,10))
        
        cm_p[:] = controlModel.getBodyPositionsGlobal() + controlModel2.getBodyPositionsGlobal()
        cm_v[:] = controlModel.getBodyVelocitiesGlobal() + controlModel2.getBodyVelocitiesGlobal()
        
#        cm_p.append(controlModel.getBodyPositionGlobal(1, (0,0,.25)))
#        cm_v.append(controlModel.getBodyVelocityGlobal(1, (0,0,.25)))
#        cm_p.append(controlModel.getBodyPositionGlobal(1, (0,0,-.25)))
#        cm_v.append(controlModel.getBodyVelocityGlobal(1, (0,0,-.25)))
        
        cm_a[:] = controlModel.getBodyAccelerationsGlobal() + controlModel2.getBodyAccelerationsGlobal()
        cm_av[:] = controlModel.getBodyAngVelocitiesGlobal() + controlModel2.getBodyAngVelocitiesGlobal()
        cm_aa[:] = controlModel.getBodyAngAccelerationsGlobal() + controlModel2.getBodyAngAccelerationsGlobal()

        mm_p[:] = motionModel.getBodyPositionsGlobal()
        mm_v[:] = motionModel.getBodyVelocitiesGlobal()
        mm_av[:] = motionModel.getBodyAngVelocitiesGlobal()
            
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()
    
def test_body_force_funcs():
    bvhFilePath = '../samples/chain_1.bvh'
#    bvhFilePath = '../samples/chain_6.bvh'
    motion = yf.readBvhFile(bvhFilePath)
    
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 100.
    mcfg.defaultBoneRatio = .8
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
    
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.gravity = (0,0,0)
    stepsPerFrame = 60
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    
    controlModel.setBodyPositionGlobal(0, (0,1,0))

    ###################################################################
    # apply force == add force
#    controlModel.applyBodyGenForceGlobal(0, (0,0,100), (0,-100,0), (0,0,0))
    # above 1 line == below 2 lines
    controlModel.applyBodyTorqueGlobal(0, (0,0,100))
    controlModel.applyBodyForceGlobal(0, (0,-100,0))
        
    p = [mm.O_Vec3()]*controlModel.getBodyNum()
    forces = [mm.O_Vec3()] * controlModel.getBodyNum()
    netForces = [mm.O_Vec3()] * controlModel.getBodyNum() 
    gravityForces = [mm.O_Vec3()] * controlModel.getBodyNum()
    
    viewer = ysv.SimpleViewer()
    viewer.record(False)
    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('model', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_LINE))

#    viewer.doc.addRenderer('forces', yr.VectorsRenderer(forces, p, (255,0,0)))
#    viewer.doc.addRenderer('netForces', yr.VectorsRenderer(netForces, p, (0,255,0)))
#    viewer.doc.addRenderer('gravityForces', yr.VectorsRenderer(gravityForces, p, (0,0,255)))
    viewer.setMaxFrame(500)

    def simulateCallback(frame):
        p[:] = controlModel.getBodyPositionsGlobal()
        forces[0] = controlModel.getBodyForceLocal(0)
        netForces[0] = controlModel.getBodyNetForceLocal(0)
        gravityForces[0] = controlModel.getBodyGravityForceLocal(0)
        
        for i in range(stepsPerFrame):
            vpWorld.step()
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()         
    
def test_penalty_model():
    bvhFilePath = '../samples/chain_1.bvh'
    motion = yf.readBvhFile(bvhFilePath)
    
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 100.
    mcfg.defaultBoneRatio = .8
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
    
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
    wcfg.gravity = (0,-9.8,0)
    stepsPerFrame = 30
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    controlModel2 = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    controlModel3 = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    
    controlModel.translateByOffset((0,1,0))
    controlModel2.translateByOffset((0,1,.5))
    controlModel3.translateByOffset((0,1,1.))
    
    bodyIDsToCheck = range(vpWorld.getBodyNum())
    mus = [1., .2, .1]
    Ks = 1000; Ds = 2*(Ks**.5)
    
    contactPositions= []    
    contactForces = []    
        
    viewer = ysv.SimpleViewer()
#    viewer.record(False)
#    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('model', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_LINE))
    viewer.doc.addRenderer('model2', cvr.VpModelRenderer(controlModel2, (255,240,255), yr.POLYGON_LINE))
    viewer.doc.addRenderer('model3', cvr.VpModelRenderer(controlModel3, (255,240,255), yr.POLYGON_LINE))
    viewer.doc.addRenderer('contactPositions', yr.PointsRenderer(contactPositions, (0,255,0), yr.POINT_POINT))
    viewer.doc.addRenderer('contactForces', yr.VectorsRenderer(contactForces, contactPositions,(0,255,0)))
    viewer.setMaxFrame(500)

    def simulateCallback(frame):
        
        for i in range(stepsPerFrame):
            controlModel.applyBodyForceGlobal(0, (1,0,0))
            controlModel2.applyBodyForceGlobal(0, (1,0,0))
            controlModel3.applyBodyForceGlobal(0, (1,0,0))
            
            # get penalty forces
            bodyIDs, positions, positionLocals, forces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, 1000, 10)

            # apply penalty forces
            vpWorld.applyPenaltyForce(bodyIDs, positionLocals, forces)
            
            vpWorld.step()
        
        contactPositions[:] = positions
        contactForces[:] = forces
            
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()         

def test_CM_CP():
#    bvhFilePath = '../samples/chain_6.bvh'
#    bvhFilePath = '../samples/block_3_rotate.bvh'
#    bvhFilePath = '../samples/block_tree_rotate.bvh'
    bvhFilePath = '../samples/chain_2.bvh'
    motion = yf.readBvhFile(bvhFilePath)
    
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 100.
    mcfg.defaultBoneRatio = .8
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
    node = mcfg.getNode('link0')
    node.density = 200.
    
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
#    wcfg.gravity = (0,0,0)
    stepsPerFrame = 30
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    
    motionModel.recordVelByFiniteDiff()
    controlModel.translateByOffset((0,.5,.5))

    CMPos_cm = [mm.O_Vec3()]
    CMVel_cm = [mm.O_Vec3()]
    CMPos_mm = [mm.O_Vec3()]
    
    CPPos_cm = [mm.O_Vec3()]
    CPVel_cm = [mm.O_Vec3()]
    
    bodyMasses = controlModel.getBodyMasses()
    totalMass = 0.
    for m in bodyMasses:
        totalMass += m
    
    bodyIDsToCheck = range(vpWorld.getBodyNum())
    mus = [.5]*len(bodyIDsToCheck)

    contactPositions= []    
    contactForces = []    

    
    viewer = ysv.SimpleViewer()
    viewer.record(False)
#    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
#    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('model', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_LINE))
    viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, (100,100,100), yr.POLYGON_LINE))

    viewer.doc.addRenderer('CMPos_cm', yr.PointsRenderer(CMPos_cm, (255,0,255)))
    viewer.doc.addRenderer('CMVel_cm', yr.VectorsRenderer(CMVel_cm, CMPos_cm, (255,0,255)))
    
    viewer.doc.addRenderer('CMPos_mm', yr.PointsRenderer(CMPos_mm, (200,0,200)))

    viewer.doc.addRenderer('CPPos_cm', yr.PointsRenderer(CPPos_cm, (0,255,0)))
    viewer.doc.addRenderer('CPVel_cm', yr.VectorsRenderer(CPVel_cm, CPPos_cm, (0,255,0)))

    viewer.doc.addRenderer('contactPositions', yr.PointsRenderer(contactPositions, (0,0,255), yr.POINT_POINT))
    viewer.doc.addRenderer('contactForces', yr.VectorsRenderer(contactForces, contactPositions,(0,0,255)))

    def simulateCallback(frame):
        controlModel.applyBodyTorqueGlobal(1, (0,10,0))
        
        for i in range(stepsPerFrame):
            bodyIDs, positions, positionLocals, forces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, 1000, 2*(1000**.5))
            vpWorld.applyPenaltyForce(bodyIDs, positionLocals, forces)
            vpWorld.step()

        del contactPositions[:]
        del contactForces[:]
        contactPositions[:] = positions
        contactForces[:] = forces
        
        motionModel.update(motion[frame])
        
        bodyPositions = controlModel.getBodyPositionsGlobal()
        bodyVelocities = controlModel.getBodyVelocitiesGlobal()
        CMPos_cm[0] = yrp.getCM(bodyPositions, bodyMasses, totalMass)
        CMVel_cm[0] = yrp.getCM(bodyVelocities, bodyMasses, totalMass)

        bodyPositions_ref = motionModel.getBodyPositionsGlobal()
        bodyVelocities_ref = motionModel.getBodyVelocitiesGlobal()
        CMPos_mm[0] = yrp.getCM(bodyPositions_ref, bodyMasses, totalMass)
        
        CPPos_cm_old = CPPos_cm[0]
        CPPos_cm[0] = yrp.getCP(positions, forces)
        
        if CPPos_cm_old==None or CPPos_cm[0]==None:
            CPVel_cm[0] = None
        else:
            CPVel_cm[0] = (CPPos_cm[0] - CPPos_cm_old)/(1./30.)

    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()
    
def test_getInternalJointOrientationsGlobal():
#    bvhFilePath = '../samples/block_tree_rotate.bvh'
    bvhFilePath = '../samples/chain_3_rotate_expt_root.bvh'
    motion = yf.readBvhFile(bvhFilePath)
    
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .8
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
    
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = -1.
    wcfg.gravity = (0,0,0)
    stepsPerFrame = 30
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()

    controlModel.translateByOffset((0,0,1))
    controlModel.fixBody(0)

    jointPositions = []
    localFrames = []
    globalFrames = []

    viewer = ysv.SimpleViewer()
    viewer.record(False)
    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('model', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_LINE))
    viewer.doc.addRenderer('localFrames', yr.FramesRenderer(localFrames, (255,0,0)))
    viewer.doc.addRenderer('globalFrames', yr.FramesRenderer(globalFrames, (0,255,0)))
    
    def simulateCallback(frame):
        controlModel.setJointAngVelocityLocal(1, (0,.5,0))
        controlModel.setJointAngVelocityLocal(2, (0,.5,0))
        
        for i in range(stepsPerFrame):
            vpWorld.step()
            
        jointPositions[:] = motion[frame].getInternalJointPositionsGlobal() + controlModel.getInternalJointPositionsGlobal()
        
        localFrames[:] = motion.getInternalJointOrientationsLocal(frame) + controlModel.getInternalJointOrientationsLocal()
        localFrames[:] = map(mm.Rp2T, localFrames, jointPositions)

        globalFrames[:] = motion[frame].getInternalJointOrientationsGlobal() + controlModel.getInternalJointOrientationsGlobal()
        globalFrames[:] = map(mm.Rp2T, globalFrames, jointPositions)
            
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()

def test_inertia_matrix():
    bvhFilePath = '../samples/chain_1.bvh'
    motion = yf.readBvhFile(bvhFilePath)
    
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 100.
    mcfg.defaultBoneRatio = 1.
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
    
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
    wcfg.gravity = (0,0,0)
    stepsPerFrame = 30
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    controlModel2 = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    
    controlModel2.translateByOffset((0,0,1))
    controlModel2.rotate(mm.exp(mm.v3(0,1,0), math.pi/2))
    
    print 'model local'
    print controlModel.getBodyInertiasLocal()
    print 
    print 'model global'
    print controlModel.getBodyInertiasGlobal()
    print
    print 'model2 global'
    print controlModel2.getBodyInertiasGlobal()

    viewer = ysv.SimpleViewer()
#    viewer.record(False)
#    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
#    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('model', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_LINE))
    viewer.doc.addRenderer('model2', cvr.VpModelRenderer(controlModel2, (255,240,255), yr.POLYGON_LINE))
    viewer.setMaxFrame(100)

    viewer.show()
    
    Fl.run()         
    
def test_myGeom():
    bvhFilePath = '../samples/chain_1.bvh'
    motion = yf.readBvhFile(bvhFilePath)
    
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 100.
    mcfg.defaultBoneRatio = .8
    for i in range(motion[0].skeleton.getElementNum()):
        node = mcfg.addNode(motion[0].skeleton.getElementName(i))
        node.geom = 'MyFoot1'
    
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
    wcfg.gravity = (0,-9.8,0)
    stepsPerFrame = 30
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    box1 = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    
    box1.translateByOffset((0,1,0))

    print len(box1.getBodyVerticesGlobal(0))
    
    bodyIDsToCheck = range(vpWorld.getBodyNum())
    mus = [1.]
    
    contactPositions= []    
    contactForces = []    
        
    viewer = ysv.SimpleViewer()
#    viewer.record(False)
#    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('box1', cvr.VpModelRenderer(box1, (255,240,255), yr.POLYGON_LINE))
    viewer.doc.addRenderer('contactPositions', yr.PointsRenderer(contactPositions, (0,255,0), yr.POINT_POINT))
    viewer.doc.addRenderer('contactForces', yr.VectorsRenderer(contactForces, contactPositions,(0,255,0)))
    viewer.setMaxFrame(500)

    def simulateCallback(frame):
        
        for i in range(stepsPerFrame):
            # get penalty forces
            bodyIDs, positions, positionLocals, forces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, 1000, 10)

            # apply penalty forces
            vpWorld.applyPenaltyForce(bodyIDs, positionLocals, forces)
            
            vpWorld.step()
        
        del contactPositions[:]
        del contactForces[:]
        contactPositions[:] = positions
        contactForces[:] = forces
            
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()         

def test_slope_box():
    bvhFilePath = '../samples/chain_1.bvh'
    motion = yf.readBvhFile(bvhFilePath)
    
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 100.
    mcfg.defaultBoneRatio = .8
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
    
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
    wcfg.gravity = (0,-9.8,0)
    stepsPerFrame = 30
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    controlModel2 = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    controlModel3 = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    
    controlModel.translateByOffset((0,1,0))
    controlModel2.translateByOffset((0,1,.5))
    controlModel3.translateByOffset((0,1,1.))
    
    bodyIDsToCheck = range(vpWorld.getBodyNum())
    mus = [1., .2, .0]
    Ks = 1000; Ds = 2*(Ks**.5)
    
    rd_box = yr.BoxesRenderer([(5., .2, 3.)], [mm.Rp2T(mm.rotZ(0.1), (0,-.5,0))], (0,0,255), yr.POLYGON_LINE)
    
    contactPositions= []    
    contactForces = []    
        
    viewer = ysv.SimpleViewer()
#    viewer.record(False)
#    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('model', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_LINE))
    viewer.doc.addRenderer('model2', cvr.VpModelRenderer(controlModel2, (255,240,255), yr.POLYGON_LINE))
    viewer.doc.addRenderer('model3', cvr.VpModelRenderer(controlModel3, (255,240,255), yr.POLYGON_LINE))
    viewer.doc.addRenderer('contactPositions', yr.PointsRenderer(contactPositions, (0,255,0), yr.POINT_POINT))
    viewer.doc.addRenderer('contactForces', yr.VectorsRenderer(contactForces, contactPositions,(0,255,0)))
    viewer.doc.addRenderer('rd_box', rd_box)
    viewer.setMaxFrame(500)

    def simulateCallback(frame):
        
        for i in range(stepsPerFrame):
#            controlModel.applyBodyForceGlobal(0, (1,0,0))
#            controlModel2.applyBodyForceGlobal(0, (1,0,0))
#            controlModel3.applyBodyForceGlobal(0, (1,0,0))
            
            # get penalty forces
#            bodyIDs, positions, positionLocals, forces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, 1000, 10)
            bodyIDs, positions, positionLocals, forces = vpWorld.calcPenaltyForce_Boxes(rd_box.boxSizes, rd_box.Ts, bodyIDsToCheck, mus, 1000, 10)

            # apply penalty forces
            vpWorld.applyPenaltyForce(bodyIDs, positionLocals, forces)
            
            vpWorld.step()
        
        contactPositions[:] = positions
        contactForces[:] = forces
            
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()         

def test_slope_character():
    bvhFilePath = '../samples/wd2_WalkSameSame01.bvh'
    motion = yf.readBvhFile(bvhFilePath)
    
    mcfgfile = open('../samples/' + 'mcfg', 'r')
    mcfg = cPickle.load(mcfgfile)
    mcfgfile.close()
    
    frameTime = 1/30.
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
    stepsPerFrame = 30
    wcfg.timeStep = (frameTime)/stepsPerFrame
    
    vpWorld = cvw.VpWorld(wcfg)
    motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    controlModel.initializeHybridDynamics()
    
    bodyIDsToCheck = range(vpWorld.getBodyNum())
    mus = [1.]*len(bodyIDsToCheck)
    
    Kt = 20.;       Dt = 2*(Kt**.5)
    Ks = 2000; Ds = 2*(Ks**.5)
    
    rd_box = yr.BoxesRenderer([(5., .2, 3.)], [mm.Rp2T(mm.rotZ(0.1), (0,-.5,0))], (0,0,255), yr.POLYGON_LINE)
    
    contactPositions= []    
    contactForces = []    
        
    viewer = ysv.SimpleViewer()
#    viewer.record(False)
#    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('model', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_LINE))
    viewer.doc.addRenderer('contactPositions', yr.PointsRenderer(contactPositions, (0,255,0), yr.POINT_POINT))
    viewer.doc.addRenderer('contactForces', yr.VectorsRenderer(contactForces, contactPositions,(0,255,0)))
    viewer.doc.addRenderer('rd_box', rd_box)
    viewer.setMaxFrame(500)

    def simulateCallback(frame):
        
        th_r = motion.getDOFPositions(frame)
        th = controlModel.getDOFPositions()
        dth_r = motion.getDOFVelocities(frame)
        dth = controlModel.getDOFVelocities()
        ddth_r = motion.getDOFAccelerations(frame)
        ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt)
        
        for i in range(stepsPerFrame):
            # get penalty forces
#            bodyIDs, positions, positionLocals, forces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, 1000, 10)
            bodyIDs, positions, positionLocals, forces = vpWorld.calcPenaltyForce_Boxes(rd_box.boxSizes, rd_box.Ts, bodyIDsToCheck, mus, 1000, 10)

            # apply penalty forces
            vpWorld.applyPenaltyForce(bodyIDs, positionLocals, forces)
            
            controlModel.setDOFAccelerations(ddth_des)
            controlModel.solveHybridDynamics()
            
            vpWorld.step()
        
        contactPositions[:] = positions
        contactForces[:] = forces
            
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()         


if __name__=='__main__':
#    test_VpMotionModel()
#    test_VpControlModel()
#    test_biped_delNode()
#    test_biped_motion_edit()
#    test_hybridDynamics()
    test_hybridDynamics_gravity()
#    test_joint_pos_vel_acc_funcs_and_tracking()
#    test_joint_force_funcs()
#    test_body_pos_vel_acc_funcs()
#    test_body_force_funcs()
#    test_penalty_model()
#    test_CM_CP()
#    test_getInternalJointOrientationsGlobal()
#    test_inertia_matrix()
#    test_myGeom()
#    test_slope_box()
#    test_slope_character()    