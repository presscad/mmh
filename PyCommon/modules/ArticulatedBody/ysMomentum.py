import numpy as np

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm

#===============================================================================
# Reference : Appendix A of Momentum Control for Balance, SIGGRAPH 2009
#===============================================================================
def make_TO(masses):
    O = np.zeros((3,3)) 
    TOs = [None]*len(masses)
    for i in range(len(masses)):
        TOs[i] = np.concatenate((mm.I_SO3()*masses[i], O), 1)
    return np.concatenate(TOs, 1)

def _make_Us(masses, positions, CM):
    Us = [None]*len(masses)
    for i in range(len(masses)):
        Us[i] = masses[i] * mm.getCrossMatrixForm(positions[i] - CM) 
#        Us[i] = masses[i] * mm.getCrossMatrixForm(CM - positions[i]) 
    return Us

# pure inertia matrix
# CM : CM or origin about angular momentum
def getPureInertiaMatrix(TO, masses, positions, CM, inertias):
    Us = _make_Us(masses, positions, CM)
    Vs = inertias
    UVs = [None]*len(masses)
    for i in range(len(masses)):
        UVs[i] = np.concatenate((Us[i], Vs[i]), 1)
    return np.concatenate((TO, np.concatenate(UVs, 1)), 0)

def make_dTO(linkNum):
    O = np.zeros((3, linkNum*3))
    return np.concatenate((O, O), 1)

def _make_dUs(masses, velocities, dCM):
    dUs = [None]*len(masses)
    for i in range(len(masses)):
        dUs[i] = masses[i] * mm.getCrossMatrixForm(velocities[i] - dCM) 
#        dUs[i] = masses[i] * mm.getCrossMatrixForm(dCM - velocities[i]) 
    return dUs

def _make_dVs(angVels, inertias):
    dVs = [None]*len(angVels)
    for i in range(len(angVels)):
        dVs[i] = np.dot(mm.getCrossMatrixForm(angVels[i]), inertias[i])
    return dVs

# time derivative of pure inertia matrix
def getPureInertiaMatrixDerivative(dTO, masses, velocities, dCM, angVels, inertias):
    dUs = _make_dUs(masses, velocities, dCM) 
    dVs = _make_dVs(angVels, inertias)
    dUVs = [None]*len(masses)
    for i in range(len(masses)):
        dUVs[i] = np.concatenate((dUs[i], dVs[i]), 1)
    return np.concatenate((dTO, np.concatenate(dUVs, 1)), 0) 

#===============================================================================
# momentum calculation by standard method
#===============================================================================
def getLinearMomentum(masses, velocities):
    L = mm.v3(0.,0.,0.)
    for i in range(len(masses)):
        L += masses[i] * velocities[i]
    return L

def getAngularMomentum(origin, inertias, angVelocities, positions, masses, velocities):
    H = mm.v3(0.,0.,0.)
    for i in range(len(masses)):
        H += np.dot(inertias[i], angVelocities[i]) + np.cross(positions[i]-origin, masses[i]*velocities[i])
    return H


if __name__=='__main__':
    import psyco; psyco.full()
    from fltk import *
    
    import Resource.ysMotionLoader as yf
    import Simulator.ysPhysConfig as ypc
    import Renderer.ysRenderer as yr
    import Renderer.csVpRenderer as cvr
    import Simulator.csVpWorld as cvw
    import Simulator.csVpModel as cvm
    import GUI.ysSimpleViewer as ysv
    import Optimization.csEQP as ceq
    import ArticulatedBody.ysJacobian as yjc
    import Util.ysPythonEx as ype
    import Motion.ysSkeletonEdit as yme
    import ArticulatedBody.ysReferencePoints as yrp

    
    def test_momentum_matrix_build():        
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
        vpWorld.initialize()

        controlModel.fixBody(0)
#        controlModel.setJointAngVelocityLocal(0, (.5,0,0))
    
        linkMasses = controlModel.getBodyMasses()
        totalMass = controlModel.getTotalMass()
        
        totalDOF = controlModel.getTotalDOF()
        jointDOFs = controlModel.getDOFs()
        J = yjc.makeEmptyJacobian(jointDOFs, controlModel.getBodyNum())
        jointPositions = [None]*totalDOF
        
        VERBOSE = True
#        VERBOSE = False
        
        if VERBOSE:
            np.set_printoptions(precision=1, linewidth=200)    
        
        TO = make_TO(linkMasses)
        dTO = make_dTO(len(linkMasses))
        if VERBOSE:
            print 'TO'
            print TO
            print 'dTO'
            print dTO
    
        viewer = ysv.SimpleViewer()
        viewer.record(False)
#        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
#        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('model', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_LINE))
        
        def simulateCallback(frame):
            linkPositions = controlModel.getBodyPositionsGlobal()
            CM = yrp.getCM(linkPositions, linkMasses, totalMass)
            inertias = controlModel.getBodyInertiasGlobal()
            P = getPureInertiaMatrix(TO, linkMasses, linkPositions, CM, inertias)
            
            linkVelocities = controlModel.getBodyVelocitiesGlobal()
            dCM = yrp.getCM(linkVelocities, linkMasses, totalMass)
            dP = getPureInertiaMatrixDerivative(dTO, linkMasses, linkVelocities, dCM, controlModel.getBodyAngVelocitiesGlobal(), inertias)
            
            if VERBOSE:
                print 'P'
                print P
                print 'dP'
                print dP

            for i in range(stepsPerFrame):
                vpWorld.step()
        viewer.setSimulateCallback(simulateCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
        
    def test_momentum_standard():        
        np.set_printoptions(precision=2, linewidth=200)    
        
        bvhFilePath = '../samples/chain_1_long.bvh'
        motion1 = yf.readBvhFile(bvhFilePath)
        
        bvhFilePath = '../samples/chain_2.bvh'
        motion2 = yf.readBvhFile(bvhFilePath)
        
        mcfg1 = ypc.ModelConfig()
        mcfg1.defaultDensity = 1000.
        mcfg1.defaultBoneRatio = 1.
        for i in range(motion1[0].skeleton.getElementNum()):
            mcfg1.addNode(motion1[0].skeleton.getElementName(i))
        mcfg2 = ypc.ModelConfig()
        mcfg2.defaultDensity = 1000.
        mcfg2.defaultBoneRatio = 1.
        for i in range(motion2[0].skeleton.getElementNum()):
            mcfg2.addNode(motion2[0].skeleton.getElementName(i))
        
        wcfg = ypc.WorldConfig()
        wcfg.planeHeight = 0.
        wcfg.gravity = (0,0,0)
        stepsPerFrame = 30
        wcfg.timeStep = (1/30.)/stepsPerFrame
        
        vpWorld = cvw.VpWorld(wcfg)
        m1 = cvm.VpControlModel(vpWorld, motion1[0], mcfg1)
        m2 = cvm.VpControlModel(vpWorld, motion2[0], mcfg2)
        vpWorld.initialize()

        force = 1000
#        force = 0
        torque = 400

        m1.translateByOffset((0,1,1))
        m1.applyBodyTorqueGlobal(0, (0,0,torque))
        m1.applyBodyForceGlobal(0, (force,0,0))
    
        m2.translateByOffset((0,1,0))
        m2.applyBodyTorqueGlobal(0, (0,0,torque/2.))
        m2.applyBodyTorqueGlobal(1, (0,0,torque/2.))
        m2.applyBodyForceGlobal(0, (force/2.,0,0))
        m2.applyBodyForceGlobal(1, (force/2.,0,0))
    
    
        masses_m1 = m1.getBodyMasses()
        masses_m2 = m2.getBodyMasses()
        totalMass_m1 = m1.getTotalMass()
        totalMass_m2 = m2.getTotalMass()
    
        p = []
        v = []

        CM = []
        L_std = []
        H_std = []
    
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('model', cvr.VpModelRenderer(m2, (255,240,255), yr.POLYGON_LINE))
        viewer.doc.addRenderer('model2', cvr.VpModelRenderer(m1, (255,240,255), yr.POLYGON_LINE))
#        viewer.doc.addRenderer('v', yr.VectorsRenderer(v, p, (0,255,0)))

        viewer.doc.addRenderer('L_std', yr.VectorsRenderer(L_std, CM, (255,0,0)))
        viewer.doc.addRenderer('H_std', yr.VectorsRenderer(H_std, CM, (255,0,0)))
        
        viewer.setMaxFrame(100)
        
        def simulateCallback(frame):
            for i in range(stepsPerFrame):
                vpWorld.step()
                
            velocities_m1 = m1.getBodyVelocitiesGlobal()
            velocities_m2 = m2.getBodyVelocitiesGlobal()
    
            positions_m1 = m1.getBodyPositionsGlobal()
            positions_m2 = m2.getBodyPositionsGlobal()
            CM_m1 = m1.getBodyPositionGlobal(0)
            CM_m2 = yrp.getCM(positions_m2, masses_m2, totalMass_m2)
            inertias_m1 = m1.getBodyInertiasGlobal()
            inertias_m2 = m2.getBodyInertiasGlobal()
            angVelocities_m1 = m1.getBodyAngVelocitiesGlobal()
            angVelocities_m2 = m2.getBodyAngVelocitiesGlobal()
                            
            L1_std = getLinearMomentum(masses_m1, velocities_m1)
            L2_std = getLinearMomentum(masses_m2, velocities_m2)
            H1_std = getAngularMomentum(CM_m1, inertias_m1, angVelocities_m1, positions_m1, masses_m1, velocities_m1)
            H2_std = getAngularMomentum(CM_m2, inertias_m2, angVelocities_m2, positions_m2, masses_m2, velocities_m2)
            
            
            #===============================================================================
            # for rendering
            #===============================================================================
            p[:] = m1.getBodyPositionsGlobal() + m2.getBodyPositionsGlobal() 
            v[:] = m1.getBodyVelocitiesGlobal() + m2.getBodyVelocitiesGlobal()
            
            CM[:] = [yrp.getCM(m1.getBodyPositionsGlobal(), m1.getBodyMasses()), CM_m2]
            L_std[:] = [L1_std, L2_std]
            H_std[:] = [H1_std, H2_std]
                
        viewer.setSimulateCallback(simulateCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
        
    def test_momentum_matrix():        
        np.set_printoptions(precision=2, linewidth=200)    
        
#        bvhFilePath = '../samples/chain_1_long.bvh'
        bvhFilePath = '../samples/chain_3_rotate.bvh'
        motion1 = yf.readBvhFile(bvhFilePath)
        
        mcfg1 = ypc.ModelConfig()
        mcfg1.defaultDensity = 1000.
        mcfg1.defaultBoneRatio = 1.
        for i in range(motion1[0].skeleton.getElementNum()):
            mcfg1.addNode(motion1[0].skeleton.getElementName(i))
        wcfg = ypc.WorldConfig()
        wcfg.planeHeight = -1.
        wcfg.gravity = (0,0,0)
        stepsPerFrame = 30
        wcfg.timeStep = (1/30.)/stepsPerFrame
        
        vpWorld = cvw.VpWorld(wcfg)
        m1 = cvm.VpControlModel(vpWorld, motion1[0], mcfg1)
        vpWorld.initialize()

        # momentum matrix information
        masses = m1.getBodyMasses()
        totalMass = m1.getTotalMass()
        TO = make_TO(masses)
        v_sol = ype.makeNestedList([6]*m1.getBodyNum())

        # jacobian for internal joints
        DOFs_internal = m1.getInternalJointDOFs()
        totalDOF_internal = m1.getTotalInternalJointDOF()

        J_internal = yjc.makeEmptyJacobian(DOFs_internal, m1.getBodyNum())
        linkJointMasks_internal = yjc.getAllLinkInternalJointMasks(motion1[0].skeleton)

        dth_flat_internal = ype.makeFlatList(totalDOF_internal)

        # momentum matrix for all joints
        DOFs_all = m1.getDOFs()
        totalDOF_all = m1.getTotalDOF()
        
        J_all = yjc.makeEmptyJacobian(DOFs_all, m1.getBodyNum())
        linkJointMasks_all = yjc.getAllLinkJointMasks(motion1[0].skeleton)
        
        dth_flat_all = ype.makeFlatList(totalDOF_all)
        
        
        p = []
        v = []
        
        rd_CM = []
        rd_L_std = []
        rd_L_jacob_internal = []
        rd_L_jacob_all = []
        rd_H_std = []
        rd_H_jacob_internal = []
        rd_H_jacob_all = []
    
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('model', cvr.VpModelRenderer(m1, (255,240,255), yr.POLYGON_LINE))
#        viewer.doc.addRenderer('v', yr.VectorsRenderer(v, p, (0,255,0)))

        viewer.doc.addRenderer('L_std', yr.VectorsRenderer(rd_L_std, rd_CM, (255,0,0)))
#        viewer.doc.addRenderer('L_jacob_internal', yr.VectorsRenderer(rd_L_jacob_internal, rd_CM, (0,255,0)))
        viewer.doc.addRenderer('L_jacob_all', yr.VectorsRenderer(rd_L_jacob_all, rd_CM, (255,255,0)))

        viewer.doc.addRenderer('H_std', yr.VectorsRenderer(rd_H_std, rd_CM, (255,0,0)))
#        viewer.doc.addRenderer('H_jacob_internal', yr.VectorsRenderer(rd_H_jacob_internal, rd_CM, (0,255,0)))
        viewer.doc.addRenderer('H_jacob_all', yr.VectorsRenderer(rd_H_jacob_all, rd_CM, (255,255,0)))
        
        viewer.setMaxFrame(100)
        
        # force
        m1.applyBodyTorqueGlobal(0, (0,0,1000))
        m1.applyBodyForceGlobal(0, (1000,0,0))
        m1.applyBodyTorqueGlobal(0, (0,1000,0))
        
        def simulateCallback(frame):
            for i in range(stepsPerFrame):
                vpWorld.step()
                
            #===============================================================================
            # momentum calculation by standard method
            #===============================================================================
            velocities = m1.getBodyVelocitiesGlobal()
            positions = m1.getBodyPositionsGlobal()
            CM = yrp.getCM(positions, masses, totalMass)
            inertias = m1.getBodyInertiasGlobal()
            angVelocities = m1.getBodyAngVelocitiesGlobal()
            
            L_std = getLinearMomentum(masses, velocities)
            H_std = getAngularMomentum(CM, inertias, angVelocities, positions, masses, velocities)
            
            #===============================================================================
            # momentum calculation by centroidal momentum matrix
            #===============================================================================
            P = getPureInertiaMatrix(TO, masses, positions, CM, inertias)

            # momentum matrix for internal joints and addition of total momentum about CM
#            jointPositions_internal = m1.getInternalJointPositionsGlobal()
#
#            Rs = m1.getInternalJointOrientationsGlobal()
#            jointAxeses_internal = [Rs[i].transpose() for i in range(0,len(Rs))]
#            
#            yjc.computeJacobian2(J_internal, DOFs_internal, jointPositions_internal, jointAxeses_internal, positions, linkJointMasks_internal)
#            
#            dth = m1.getInternalJointAngVelocitiesLocal()
#            ype.flatten(dth, dth_flat_internal)
# 
#            PJ_internal = np.dot(P, J_internal)
#            LH_internal = np.dot(PJ_internal, dth_flat_internal)
#            L2_jacob_internal, H2_jacob_internal = np.hsplit(LH_internal, 2)
#            
#            p_root = m1.getBodyPositionGlobal(0)
#            v_root = m1.getBodyVelocityGlobal(0)
#            w_root = m1.getBodyAngVelocityGlobal(0)
#            
#            L_jacob_internal = mm.v3(0.,0.,0.)
#            L_jacob_internal += (totalMass * v_root)
#            L_jacob_internal += (-totalMass * np.cross( (CM - p_root), w_root))
#
            L_jacob_internal = None
            H_jacob_internal = None

            # momentum matrix for all joints
            jointPositions_all = m1.getJointPositionsGlobal()
            jointAxeses_all = m1.getDOFAxeses()
            
            yjc.computeJacobian2(J_all, DOFs_all, jointPositions_all, jointAxeses_all, positions, linkJointMasks_all)
            
            dth = m1.getDOFVelocities()
            ype.flatten(dth, dth_flat_all)
            
            PJ_all= np.dot(P, J_all)
            LH_all= np.dot(PJ_all, dth_flat_all)
            L_jacob_all, H_jacob_all= np.hsplit(LH_all, 2)

            #===============================================================================
            # for rendering
            #===============================================================================
            p[:] = positions 
            v[:] = velocities
            
            rd_CM[:] = [CM]
            rd_L_std[:] = [L_std]
            rd_L_jacob_internal[:] = [L_jacob_internal]
            rd_L_jacob_all[:] = [L_jacob_all]
            
            rd_H_std[:] = [H_std]
            rd_H_jacob_internal[:] = [H_jacob_internal]
            rd_H_jacob_all[:] = [H_jacob_all]

                
        viewer.setSimulateCallback(simulateCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
        
    pass
#    test_momentum_matrix_build()
#    test_momentum_standard()
    test_momentum_matrix()